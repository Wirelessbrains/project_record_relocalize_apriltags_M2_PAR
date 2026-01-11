import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
import sys
import os
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline, PchipInterpolator

# ==========================================
# 1. SETUP E CARREGAMENTO DE DADOS
# ==========================================
print("Carregando dataset de medições...")
filename = 'circle_dense_truth.yaml'
try:
    with open(filename, 'r') as f:
        dataset = yaml.safe_load(f)
except Exception as e:
    print(f"Erro ao abrir arquivo '{filename}': {e}"); sys.exit(1)

tag_id_map, next_tag_idx, reverse_tag_map = {}, 0, {}
obs_cam_idxs, obs_tag_idxs, obs_measurements, ground_truth_traj = [], [], [], []
frames = dataset['frames']
num_frames = len(frames)
first_observation_data = None

for i, frame in enumerate(frames):
    curr_p, curr_q = frame['pose']['position'], frame['pose']['orientation']
    ground_truth_traj.append(np.array([curr_p['x'], curr_p['y'], curr_p['z']]))
    R_c_input = R.from_quat([curr_q['x'], curr_q['y'], curr_q['z'], curr_q['w']])
    detection_field = frame.get('detections') or frame.get('learned') or frame.get('tags')
    if detection_field:
        if first_observation_data is None:
            first_tag_name = list(detection_field.keys())[0]
            first_tag_pos_dict = detection_field[first_tag_name]['position']
            first_meas = np.array([first_tag_pos_dict['x'], first_tag_pos_dict['y'], first_tag_pos_dict['z']])
            first_observation_data = {'cam_gt_rot': R_c_input, 'meas_local': first_meas, 'tag_name': first_tag_name}
        for tag_name, pose_data in detection_field.items():
            if tag_name not in tag_id_map:
                tag_id_map[tag_name] = next_tag_idx
                reverse_tag_map[next_tag_idx] = tag_name
                next_tag_idx += 1
            obs_cam_idxs.append(i)
            obs_tag_idxs.append(tag_id_map[tag_name])
            obs_measurements.append(np.array([pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z']]))

obs_cam_idxs, obs_tag_idxs, obs_measurements, ground_truth_traj = np.array(obs_cam_idxs, dtype=np.int32), np.array(obs_tag_idxs, dtype=np.int32), np.array(obs_measurements, dtype=np.float64), np.array(ground_truth_traj)
num_tags, num_observations = len(tag_id_map), len(obs_cam_idxs)
print(f"Dados Carregados: {num_frames} frames, {num_tags} tags, {num_observations} observações.")

# ==========================================
# 2. INICIALIZAÇÃO E OTIMIZAÇÃO DE TAGS (Passo 1)
# ==========================================
print("\n--- PASSO 1: Otimizando Posições das Tags ---")
P_tag_anchor_world = np.zeros(3)
R_cam_anchor, P_tag_local = first_observation_data['cam_gt_rot'], first_observation_data['meas_local']
start_pos_vec = P_tag_anchor_world - R_cam_anchor.apply(P_tag_local)
r_vec_start = R_cam_anchor.as_rotvec()

x0_cam_pos, x0_cam_rot = np.zeros((num_frames, 3)), np.zeros((num_frames, 3))
for i, frame in enumerate(frames):
    p, q = frame['pose']['position'], frame['pose']['orientation']
    x0_cam_pos[i], x0_cam_rot[i] = [p['x'], p['y'], p['z']], R.from_quat([q['x'], q['y'], q['z'], q['w']]).as_rotvec()

idx_start = obs_cam_idxs[0]
R_offset_obj = R.from_rotvec(r_vec_start) * R.from_rotvec(x0_cam_rot[idx_start]).inv()
x0_cam_pos_aligned, x0_cam_rot_aligned = np.copy(x0_cam_pos), np.copy(x0_cam_rot)
for i in range(num_frames):
    x0_cam_pos_aligned[i] = R_offset_obj.apply(x0_cam_pos[i] - x0_cam_pos[idx_start]) + start_pos_vec
    x0_cam_rot_aligned[i] = (R_offset_obj * R.from_rotvec(x0_cam_rot[i])).as_rotvec()

tag_projections = [[] for _ in range(num_tags)]
for i in range(num_observations):
    R_cam, t_cam = R.from_rotvec(x0_cam_rot_aligned[obs_cam_idxs[i]]), x0_cam_pos_aligned[obs_cam_idxs[i]]
    tag_projections[obs_tag_idxs[i]].append(t_cam + R_cam.apply(obs_measurements[i]))
x0_tag_pos = np.zeros((num_tags, 3))
for t_idx in range(num_tags):
    if tag_projections[t_idx]: x0_tag_pos[t_idx] = np.median(tag_projections[t_idx], axis=0)
anchor_tag_idx = tag_id_map[first_observation_data['tag_name']]
x0_tag_pos[anchor_tag_idx] = P_tag_anchor_world

def fun_tags_only(x_tags, c_pos, c_rot, anchor_idx):
    t_pos = x_tags.reshape(num_tags, 3)
    res_obs = (R.from_rotvec(c_rot[obs_cam_idxs]).inv().apply(t_pos[obs_tag_idxs] - c_pos[obs_cam_idxs]) - obs_measurements).ravel()
    res_anchor = (t_pos[anchor_idx] - P_tag_anchor_world).ravel() * 100.0
    return np.concatenate([res_obs, res_anchor])

res_step1 = least_squares(fun_tags_only, x0_tag_pos.ravel(), loss='soft_l1', f_scale=1.0, verbose=0, args=(x0_cam_pos_aligned, x0_cam_rot_aligned, anchor_tag_idx))
est_tag_pos = res_step1.x.reshape(num_tags, 3)
print("Otimização das tags concluída.")

# ==========================================
# 3. OTIMIZAÇÃO DA TRAJETÓRIA DA CÂMERA (Passo 2)
# ==========================================
print("\n--- PASSO 2: Otimizando a Trajetória da Câmera (com tags fixas e suavidade) ---")
def fun_cam_only(x_cams, num_cams, t_pos_fixed, anchor_pos, anchor_rot, smooth_weight=50.0):
    cam_poses_rot = x_cams[0:num_cams*3].reshape((num_cams, 3))
    cam_poses_pos = x_cams[num_cams*3:].reshape((num_cams, 3))
    
    # 1. Erro de Reprojeção (Visual)
    batch_cam_pos = cam_poses_pos[obs_cam_idxs]
    batch_cam_rot = R.from_rotvec(cam_poses_rot[obs_cam_idxs])
    batch_tag_pos = t_pos_fixed[obs_tag_idxs]
    res_obs = (batch_cam_rot.inv().apply(batch_tag_pos - batch_cam_pos) - obs_measurements).ravel()
    
    # 2. Âncora (Fixar início)
    res_anchor_pos = (cam_poses_pos[0] - anchor_pos) * 100.0
    res_anchor_rot = (cam_poses_rot[0] - anchor_rot) * 100.0
    
    # 3. Regularização de Suavidade (Aceleração)
    # Penaliza (pos[i+1] - pos[i]) - (pos[i] - pos[i-1]) => pos[i+1] - 2*pos[i] + pos[i-1]
    # Isso força velocidade constante e remove spikes (aceleração infinita)
    res_smooth = np.array([])
    if num_cams > 2:
        accel = cam_poses_pos[:-2] - 2*cam_poses_pos[1:-1] + cam_poses_pos[2:]
        res_smooth = accel.ravel() * smooth_weight

    return np.concatenate([res_obs, res_anchor_pos, res_anchor_rot, res_smooth])

x0_cams = np.concatenate([x0_cam_rot_aligned.ravel(), x0_cam_pos_aligned.ravel()])
anchor_pos_cam, anchor_rot_cam = x0_cam_pos_aligned[0], x0_cam_rot_aligned[0]

# Nota: Removemos jac_sparsity manual pois a suavidade cria dependências complexas entre frames.
# O solver estimará a jacobiana ou usará aproximação densa. Para <2000 frames é aceitável.
print("Construindo matriz de esparsidade para otimização rápida...")
num_cam_vars = num_frames * 6
num_obs_residuals = num_observations * 3
num_anchor_residuals = 6
num_smooth_residuals = (num_frames - 2) * 3 if num_frames > 2 else 0
total_residuals = num_obs_residuals + num_anchor_residuals + num_smooth_residuals

sparcity_step2 = lil_matrix((total_residuals, num_cam_vars), dtype=int)

# 1. Esparsidade das Observações
row_idx = 0
for i in range(num_observations):
    cam_idx = obs_cam_idxs[i]
    col_start_rot, col_start_pos = cam_idx * 3, num_frames * 3 + cam_idx * 3
    sparcity_step2[row_idx:row_idx+3, col_start_rot:col_start_rot+3] = 1
    sparcity_step2[row_idx:row_idx+3, col_start_pos:col_start_pos+3] = 1
    row_idx += 3

# 2. Esparsidade da Âncora
sparcity_step2[row_idx:row_idx+6, 0:3] = 1
sparcity_step2[row_idx:row_idx+6, num_frames*3 : num_frames*3+3] = 1
row_idx += 6

# 3. Esparsidade da Suavidade
# Termo (pos[i+1] - 2*pos[i] + pos[i-1]) afeta pos[i-1], pos[i], pos[i+1]
pos_offset = num_frames * 3
if num_frames > 2:
    for i in range(1, num_frames - 1): # i percorre os frames centrais da janela de 3
        # Indices das variaveis de posicao para i-1, i, i+1
        idx_prev = pos_offset + (i - 1) * 3
        idx_curr = pos_offset + (i) * 3
        idx_next = pos_offset + (i + 1) * 3
        
        # O residuo atual (3 linhas x,y,z) depende dessas 3 poses
        sparcity_step2[row_idx:row_idx+3, idx_prev:idx_prev+3] = 1
        sparcity_step2[row_idx:row_idx+3, idx_curr:idx_curr+3] = 1
        sparcity_step2[row_idx:row_idx+3, idx_next:idx_next+3] = 1
        row_idx += 3

print("Iniciando otimização robusta e rápida...")
res_step2 = least_squares(fun_cam_only, x0_cams, jac_sparsity=sparcity_step2, verbose=1, method='trf', loss='soft_l1', f_scale=1.0, x_scale='jac', args=(num_frames, est_tag_pos, anchor_pos_cam, anchor_rot_cam))
x_opt_cam = res_step2.x
opt_cam_pos = x_opt_cam[num_frames*3:].reshape((num_frames, 3))
print("Otimização da trajetória da câmera finalizada.")

# ==========================================
# 4. ANÁLISE FINAL, TABELA DE ERROS E PLOTAGEM
# ==========================================
print("\n--- PASSO 3: Análise Final e Plotagem ---")

# --- Análise de Erro das Tags ---
gt_tags_file = 'circle_tags_scenario_dense.yaml'
gt_tags_map = {}
if os.path.exists(gt_tags_file):
    with open(gt_tags_file, 'r') as f:
        gt_data = yaml.safe_load(f)
        for k, v in gt_data.get('tags', gt_data).items(): gt_tags_map[k] = np.array(list(v['position'].values()))

R_tag_align, t_tag_align = np.eye(3), np.zeros(3)
src_pts_tags = np.array([est_tag_pos[tag_id_map[name]] for name in gt_tags_map if name in tag_id_map])
dst_pts_tags = np.array([pos for name, pos in gt_tags_map.items() if name in tag_id_map])
if len(src_pts_tags) >= 3:
    cent_src, cent_dst = np.mean(src_pts_tags, axis=0), np.mean(dst_pts_tags, axis=0)
    H = (src_pts_tags - cent_src).T @ (dst_pts_tags - cent_dst)
    U, S, Vt = np.linalg.svd(H)
    R_tag_align = Vt.T @ U.T
    if np.linalg.det(R_tag_align) < 0: Vt[2,:] *= -1; R_tag_align = Vt.T @ U.T
    t_tag_align = cent_dst - R_tag_align @ cent_src
est_tag_pos_aligned = (R_tag_align @ est_tag_pos.T).T + t_tag_align

print("\n" + "="*70); print(f"{ 'ANÁLISE DETALHADA DO ERRO DAS TAGS':^70}"); print("="*70)
print(f"| {'Tag':<12} | {'dX (m)':<10} | {'dY (m)':<10} | {'dZ (m)':<10} | {'Total (m)':<12} |"); print("-" * 70)
tag_errors = []
for i in range(num_tags):
    name = reverse_tag_map[i]
    if name in gt_tags_map:
        err_vec = est_tag_pos_aligned[i] - gt_tags_map[name]
        total_err = np.linalg.norm(err_vec)
        tag_errors.append(total_err)
        print(f"| {name:<12} | {err_vec[0]:<+10.4f} | {err_vec[1]:<+10.4f} | {err_vec[2]:<+10.4f} | {total_err:<12.4f} |")
print("-" * 70)
if tag_errors: print(f"Resumo Erro Tags -> Média: {np.mean(tag_errors):.4f} m | Máx: {np.max(tag_errors):.4f} m | Mín: {np.min(tag_errors):.4f} m")

# --- Análise de Erro da Trajetória (ATE) ---
src_traj, dst_traj = opt_cam_pos, ground_truth_traj
cent_src_traj, cent_dst_traj = np.mean(src_traj, axis=0), np.mean(dst_traj, axis=0)
H_traj = (src_traj - cent_src_traj).T @ (dst_traj - cent_dst_traj)
U_traj, _, Vt_traj = np.linalg.svd(H_traj)
R_shape_align, t_shape_align = Vt_traj.T @ U_traj.T, cent_dst_traj - (Vt_traj.T @ U_traj.T) @ cent_src_traj
final_pos_aligned_for_shape = (R_shape_align @ opt_cam_pos.T).T + t_shape_align

# Generate interpolated GT trajectory for plotting from the sparse optimized trajectory points
# Improved sampling: ensure start and end points are included to cover the full trajectory
num_total_poses = len(final_pos_aligned_for_shape)
step_size = 5
indices = np.arange(0, num_total_poses, step_size)
if indices[-1] != num_total_poses - 1:
    indices = np.append(indices, num_total_poses - 1)
sparse_optimized_poses = final_pos_aligned_for_shape[indices]

# --- Outlier Filtering for sparse_optimized_poses ---
filtered_sparse_optimized_poses = []
filtered_indices = []
window_size = 5 # Number of points to consider for local median, should be odd
threshold_distance = 0.2 # meters, mais rigoroso para evitar desvios locais

if len(sparse_optimized_poses) > window_size:
    for i in range(len(sparse_optimized_poses)):
        # Always keep first and last points to ensure start/end of trajectory are present
        if i == 0 or i == len(sparse_optimized_poses) - 1:
            filtered_sparse_optimized_poses.append(sparse_optimized_poses[i])
            filtered_indices.append(indices[i])
            continue

        # Define window for median calculation
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(sparse_optimized_poses), i + window_size // 2 + 1)
        
        local_window = sparse_optimized_poses[start_idx:end_idx]
        local_median = np.median(local_window, axis=0)
        
        current_point = sparse_optimized_poses[i]
        distance_from_median = np.linalg.norm(current_point - local_median)
        
        if distance_from_median < threshold_distance:
            filtered_sparse_optimized_poses.append(current_point)
            filtered_indices.append(indices[i])
    filtered_sparse_optimized_poses = np.array(filtered_sparse_optimized_poses)
    filtered_indices = np.array(filtered_indices)
else:
    filtered_sparse_optimized_poses = sparse_optimized_poses # Not enough points to filter
    filtered_indices = indices

# Use filtered points for interpolation
if len(filtered_sparse_optimized_poses) > 1:
    x_coords_sparse = filtered_sparse_optimized_poses[:, 0]
    y_coords_sparse = filtered_sparse_optimized_poses[:, 1]
    z_coords_sparse = filtered_sparse_optimized_poses[:, 2]

    # Create a parameter 't' for interpolation using the original indices
    t_sparse = filtered_indices

    # Create PchipInterpolator for each dimension (stable, no overshoots)
    cs_x = PchipInterpolator(t_sparse, x_coords_sparse)
    cs_y = PchipInterpolator(t_sparse, y_coords_sparse)
    cs_z = PchipInterpolator(t_sparse, z_coords_sparse)

    # Generate a denser set of points for the interpolated trajectory
    # Interpolate strictly from the first to the last filtered index
    t_interp = np.linspace(t_sparse[0], t_sparse[-1], num_frames * 2) 
    interpolated_gt_for_plot = np.array([cs_x(t_interp), cs_y(t_interp), cs_z(t_interp)]).T
else:
    interpolated_gt_for_plot = final_pos_aligned_for_shape # If not enough points, use original (dense) optimized trajectory


# --- Análise de Erro da Trajetória Interpolada ---
if len(filtered_sparse_optimized_poses) > 1:
    # Avaliar a interpolação em TODOS os índices de frames válidos cobertos pela spline
    all_indices = np.arange(num_frames)
    valid_mask = (all_indices >= t_sparse[0]) & (all_indices <= t_sparse[-1])
    valid_eval_indices = all_indices[valid_mask]
    
    interp_full_traj = np.column_stack((
        cs_x(valid_eval_indices),
        cs_y(valid_eval_indices),
        cs_z(valid_eval_indices)
    ))
    
    gt_subset = ground_truth_traj[valid_mask]
    
    traj_errors_vec = interp_full_traj - gt_subset
    traj_errors_dist = np.linalg.norm(traj_errors_vec, axis=1)
    
    traj_mae = np.mean(traj_errors_dist)
    traj_rmse = np.sqrt(np.mean(traj_errors_dist**2))
    traj_max = np.max(traj_errors_dist)
    traj_min = np.min(traj_errors_dist)
    
    print("\n" + "="*70); print(f"{ 'ANÁLISE DE ERRO DA TRAJETÓRIA (INTERPOLADA vs GT)':^70}"); print("="*70)
    print(f"Comparação em {len(valid_eval_indices)} frames:")
    print(f"| {'Métrica':<20} | {'Valor (m)':<15} |")
    print("-" * 43)
    print(f"| {'Erro Médio (MAE)':<20} | {traj_mae:<15.4f} |")
    print(f"| {'RMSE':<20} | {traj_rmse:<15.4f} |")
    print(f"| {'Erro Máximo':<20} | {traj_max:<15.4f} |")
    print(f"| {'Erro Mínimo':<20} | {traj_min:<15.4f} |")
    print("-" * 43)
else:
    # Fallback para trajetória bruta se interpolação falhar
    shape_errors = np.linalg.norm(ground_truth_traj - final_pos_aligned_for_shape, axis=1)
    shape_ate_rmse = np.sqrt(np.mean(shape_errors**2))
    print(f"\nErro de Trajetória (Bruta ATE RMSE): {shape_ate_rmse:.4f} m (Interpolação insuficiente)")

# --- Plotagem Final Única e Completa ---
print("\nGerando gráfico final com todas as informações...")
plt.figure(figsize=(13, 11))
ax = plt.gca()

# Alinhar todos os dados para o frame do plot (que é o frame do ground_truth_traj)
est_tag_pos_plot_aligned = (R_shape_align @ est_tag_pos.T).T + t_shape_align
if gt_tags_map:
    # Transformar gt_tags para o frame do plot é complexo. A melhor aproximação é alinhar ao est_tag_pos
    R_gt_tags_to_est, t_gt_tags_to_est = np.eye(3), np.zeros(3)
    src_gt_tags = np.array([gt_tags_map[name] for name in gt_tags_map if name in tag_id_map])
    dst_est_tags = np.array([est_tag_pos_aligned[tag_id_map[name]] for name in gt_tags_map if name in tag_id_map])
    if len(src_gt_tags) >= 3:
      cent_src_gt, cent_dst_est = np.mean(src_gt_tags, axis=0), np.mean(dst_est_tags, axis=0)
      H_gt = (src_gt_tags - cent_src_gt).T @ (dst_est_tags - cent_dst_est)
      U_gt, _, Vt_gt = np.linalg.svd(H_gt)
      R_gt_tags_to_est = Vt_gt.T @ U_gt.T
      t_gt_tags_to_est = cent_dst_est - R_gt_tags_to_est @ cent_src_gt
    
    gt_tags_for_plot = (R_gt_tags_to_est @ np.array(list(gt_tags_map.values())).T).T + t_gt_tags_to_est
else:
    gt_tags_for_plot = np.array([])


# Trajetórias
ax.plot(ground_truth_traj[:, 0], ground_truth_traj[:, 1], 'k--', linewidth=2, label='Trajetória de Referência (GT)')
ax.plot(interpolated_gt_for_plot[:, 0], interpolated_gt_for_plot[:, 1], 'g-', linewidth=2.5, label='Trajetória Otimizada (Interpolada)')

# Poses da Câmera Otimizadas (esparsas)
ax.scatter(final_pos_aligned_for_shape[::20, 0], final_pos_aligned_for_shape[::20, 1], marker='^', facecolors='none', edgecolors='blue', s=80, label='Poses Otimizadas (Esparso)')

# Tags
ax.scatter(est_tag_pos_plot_aligned[:,0], est_tag_pos_plot_aligned[:,1], c='red', marker='x', s=120, zorder=5, label='Tags Estimadas (Alinhadas)')
if gt_tags_map:
    ax.scatter(gt_tags_for_plot[:,0], gt_tags_for_plot[:,1], facecolors='none', edgecolors='orange', s=200, linewidth=2.5, zorder=4, label='Tags Reais (Alinhadas)')

ax.set_title("Análise Definitiva da Otimização em 2 Passos")
ax.set_xlabel("X (m)"), ax.set_ylabel("Y (m)")
ax.legend(loc='best'), ax.grid(True, linestyle='--'), ax.set_aspect('equal', 'box')
plt.tight_layout()
plt.savefig("definitive_2_step_analysis.png")
print("\nGráfico definitivo salvo como 'definitive_2_step_analysis.png'")
plt.show()