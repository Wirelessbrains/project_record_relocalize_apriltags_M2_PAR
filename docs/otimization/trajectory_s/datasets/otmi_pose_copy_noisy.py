import yaml
import numpy as np
import sys
import os
import matplotlib

# Try to use an interactive backend when available so the plots open for zoom/pan.
if matplotlib.get_backend().lower() in ("agg", "cairoagg", "svg"):
    try:
        matplotlib.use("TkAgg", force=True)
    except Exception:
        pass  # Fall back to current backend if Tk is unavailable

import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline, splrep, splev

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
        print(f"Anchor Tag Name: {first_observation_data['tag_name']}")
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

# Adicionando ruído gaussiano às poses iniciais da câmera
std_dev_cam_pos_noise = 0.05  # 5 cm
std_dev_cam_rot_noise = 0.05 # 0.05 rad (aprox. 2.8 graus)
x0_cam_pos += np.random.normal(0, std_dev_cam_pos_noise, x0_cam_pos.shape)
x0_cam_rot += np.random.normal(0, std_dev_cam_rot_noise, x0_cam_rot.shape)

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
x0_tag_pos[anchor_tag_idx] = P_tag_anchor_world # Set anchor tag to fixed position before adding noise to all tags

# Adicionando ruído gaussiano à pose inicial das tags para simular incerteza
std_dev_tag_noise = 0.05  # 5 cm de desvio padrão
x0_tag_pos += np.random.normal(0, std_dev_tag_noise, x0_tag_pos.shape)

def fun_tags_only(x_tags, c_pos, c_rot, anchor_idx):
    t_pos = x_tags.reshape(num_tags, 3)
    res_obs = (R.from_rotvec(c_rot[obs_cam_idxs]).inv().apply(t_pos[obs_tag_idxs] - c_pos[obs_cam_idxs]) - obs_measurements).ravel() * 10.0
    res_anchor = (t_pos[anchor_idx] - P_tag_anchor_world).ravel() * 100.0
    return np.concatenate([res_obs, res_anchor])

res_step1 = least_squares(fun_tags_only, x0_tag_pos.ravel(), verbose=0, args=(x0_cam_pos_aligned, x0_cam_rot_aligned, anchor_tag_idx))
est_tag_pos = res_step1.x.reshape(num_tags, 3)
print("Otimização das tags concluída.")

# ==========================================
# 3. OTIMIZAÇÃO DA TRAJETÓRIA DA CÂMERA (Passo 2)
# ==========================================
print("\n--- PASSO 2: Otimizando a Trajetória da Câmera (com tags fixas) ---")
def fun_cam_only(x_cams, num_cams, t_pos_fixed, anchor_pos, anchor_rot):
    cam_poses_rot = x_cams[0:num_cams*3].reshape((num_cams, 3))
    cam_poses_pos = x_cams[num_cams*3:].reshape((num_cams, 3))
    batch_cam_pos = cam_poses_pos[obs_cam_idxs]
    batch_cam_rot = R.from_rotvec(cam_poses_rot[obs_cam_idxs])
    batch_tag_pos = t_pos_fixed[obs_tag_idxs]
    
    res_obs = (batch_cam_rot.inv().apply(batch_tag_pos - batch_cam_pos) - obs_measurements).ravel() * 20.0 
    res_anchor_pos = (cam_poses_pos[0] - anchor_pos) * 1000.0
    res_anchor_rot = (cam_poses_rot[0] - anchor_rot) * 1000.0
    res_loop_pos = (cam_poses_pos[num_cams-1] - cam_poses_pos[0]).ravel() * 100.0
    res_loop_rot = (R.from_rotvec(cam_poses_rot[num_cams-1]) * R.from_rotvec(cam_poses_rot[0]).inv()).as_rotvec().ravel() * 100.0

    res_smooth_pos = (cam_poses_pos[1:] - cam_poses_pos[:-1]).ravel() * 15.0
    res_smooth_rot = (R.from_rotvec(cam_poses_rot[1:]) * R.from_rotvec(cam_poses_rot[:-1]).inv()).as_rotvec().ravel() * 15.0
    
    res_accel_pos = (cam_poses_pos[2:] - 2*cam_poses_pos[1:-1] + cam_poses_pos[:-2]).ravel() * 30.0
    
    return np.concatenate([res_obs, res_anchor_pos, res_anchor_rot, res_loop_pos, res_loop_rot, res_smooth_pos, res_smooth_rot, res_accel_pos])

x0_cams = np.concatenate([x0_cam_rot_aligned.ravel(), x0_cam_pos_aligned.ravel()])
anchor_pos_cam, anchor_rot_cam = x0_cam_pos_aligned[0], x0_cam_rot_aligned[0]

num_cam_vars = num_frames * 6
num_smooth_residuals = (num_frames - 1) * 6
num_accel_residuals = (num_frames - 2) * 3
num_residuals = num_observations * 3 + 6 + 6 + num_smooth_residuals + num_accel_residuals

sparcity_step2 = lil_matrix((num_residuals, num_cam_vars), dtype=int)
row_idx = 0
for i in range(num_observations):
    cam_idx = obs_cam_idxs[i]
    col_start_rot, col_start_pos = cam_idx * 3, num_frames * 3 + cam_idx * 3
    sparcity_step2[row_idx:row_idx+3, col_start_rot:col_start_rot+3] = 1
    sparcity_step2[row_idx:row_idx+3, col_start_pos:col_start_pos+3] = 1
    row_idx += 3
sparcity_step2[row_idx:row_idx+6, 0:3] = 1
sparcity_step2[row_idx:row_idx+6, num_frames*3 : num_frames*3+3] = 1
row_idx += 6
sparcity_step2[row_idx:row_idx+3, num_frames*3 + (num_frames-1)*3 : num_frames*3 + (num_frames-1)*3 + 3] = 1
sparcity_step2[row_idx:row_idx+3, num_frames*3 : num_frames*3+3] = 1
row_idx += 3
sparcity_step2[row_idx:row_idx+3, (num_frames-1)*3 : (num_frames-1)*3 + 3] = 1
sparcity_step2[row_idx:row_idx+3, 0:3] = 1
row_idx += 3
for i in range(num_frames - 1):
    sparcity_step2[row_idx:row_idx+3, num_frames*3 + i*3 : num_frames*3 + i*3 + 3] = 1
    sparcity_step2[row_idx:row_idx+3, num_frames*3 + (i+1)*3 : num_frames*3 + (i+1)*3 + 3] = 1
    row_idx += 3
    sparcity_step2[row_idx:row_idx+3, i*3 : i*3 + 3] = 1
    sparcity_step2[row_idx:row_idx+3, (i+1)*3 : (i+1)*3 + 3] = 1
    row_idx += 3
for i in range(num_frames - 2):
    sparcity_step2[row_idx:row_idx+3, num_frames*3 + i*3 : num_frames*3 + (i+3)*3] = 1
    row_idx += 3

res_step2 = least_squares(fun_cam_only, x0_cams, jac_sparsity=sparcity_step2, verbose=0, method='trf', loss='huber', f_scale=0.1, x_scale='jac', args=(num_frames, est_tag_pos, anchor_pos_cam, anchor_rot_cam))
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

# Generate interpolated GT trajectory for plotting from the optimized trajectory points
# Generate interpolated GT trajectory for plotting from the sparse optimized trajectory points
sparse_optimized_poses = final_pos_aligned_for_shape[::2]

# --- Outlier Filtering for sparse_optimized_poses ---
filtered_sparse_optimized_poses = []
window_size = 7 # Increased window size for more robust median calculation
threshold_distance = 0.02 # meters, significantly reduced to aggressively filter outliers

if len(sparse_optimized_poses) > window_size:
    for i in range(len(sparse_optimized_poses)):
        # Define window for median calculation
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(sparse_optimized_poses), i + window_size // 2 + 1)
        
        local_window = sparse_optimized_poses[start_idx:end_idx]
        local_median = np.median(local_window, axis=0)
        
        current_point = sparse_optimized_poses[i]
        distance_from_median = np.linalg.norm(current_point - local_median)
        
        if distance_from_median < threshold_distance:
            filtered_sparse_optimized_poses.append(current_point)
    filtered_sparse_optimized_poses = np.array(filtered_sparse_optimized_poses)
else:
    filtered_sparse_optimized_poses = sparse_optimized_poses # Not enough points to filter

# Use filtered points for interpolation
if len(filtered_sparse_optimized_poses) > 1:
    x_coords_sparse = filtered_sparse_optimized_poses[:, 0]
    y_coords_sparse = filtered_sparse_optimized_poses[:, 1]
    z_coords_sparse = filtered_sparse_optimized_poses[:, 2]

    # Create a parameter 't' for interpolation (e.g., cumulative distance or simply index)
    t_sparse = np.arange(len(filtered_sparse_optimized_poses))

    # Create spline representations for each dimension
    tck_x = splrep(t_sparse, x_coords_sparse, s=0.01) # Larger s for more smoothing
    tck_y = splrep(t_sparse, y_coords_sparse, s=0.01) # Larger s for more smoothing
    tck_z = splrep(t_sparse, z_coords_sparse, s=0.01) # Larger s for more smoothing

    # Generate a denser set of points for the interpolated trajectory
    # Interpolate to a much higher density for smoother appearance
    t_interp = np.linspace(0, len(filtered_sparse_optimized_poses) - 1, num_frames * 50) 
    interpolated_gt_for_plot = np.array([splev(t_interp, tck_x), splev(t_interp, tck_y), splev(t_interp, tck_z)]).T
else:
    interpolated_gt_for_plot = final_pos_aligned_for_shape # If not enough points, use original (dense) optimized trajectory


shape_errors = np.linalg.norm(ground_truth_traj - final_pos_aligned_for_shape, axis=1)
shape_ate_rmse = np.sqrt(np.mean(shape_errors**2))

print("\n" + "="*70); print(f"{ 'ANÁLISE DE ERRO DA TRAJETÓRIA (FORMA)':^70}"); print("="*70)
print(f"Erro de Trajetória (ATE RMSE): {shape_ate_rmse:.4f} m")

# --- Plotagem Final (agora dividida em gráficos separados) ---
print("\nGerando gráficos finais separados para melhor visualização...")
# Alinhar todos os dados para o frame do plot (que é o frame do ground_truth_traj)
est_tag_pos_plot_aligned = (R_shape_align @ est_tag_pos.T).T + t_shape_align
if gt_tags_map:
    # Alinha GT das tags ao frame das tags estimadas para plot dedicado
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

# Figura 1: Trajetórias e poses da câmera
plt.figure(figsize=(12, 10))
ax1 = plt.gca()
ax1.plot(ground_truth_traj[:, 0], ground_truth_traj[:, 1], 'k--', linewidth=2, label='Trajetória de Referência (GT)')
ax1.plot(final_pos_aligned_for_shape[:, 0], final_pos_aligned_for_shape[:, 1], 'g-', linewidth=2.5, label='Trajetória Otimizada')
ax1.plot(interpolated_gt_for_plot[:, 0], interpolated_gt_for_plot[:, 1], 'y--', linewidth=1.5, label='Interp. dos Triângulos Otimizados (Suavizada)')
ax1.scatter(final_pos_aligned_for_shape[::10, 0], final_pos_aligned_for_shape[::10, 1], marker='^', facecolors='none', edgecolors='blue', s=80, label='Poses Otimizadas (Esparsas)')
x0_cam_pos_aligned_for_plot = (R_shape_align @ x0_cam_pos.T).T + t_shape_align
ax1.scatter(x0_cam_pos_aligned_for_plot[:, 0], x0_cam_pos_aligned_for_plot[:, 1], c='purple', marker='+', s=45, alpha=0.6, label='Poses Iniciais da Câmera (Com Ruído)')
ax1.set_title("Trajetórias e Poses da Câmera")
ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)")
ax1.legend(loc='best'); ax1.grid(True, linestyle='--'); ax1.set_aspect('equal', 'box')
plt.tight_layout()
plt.savefig("definitive_trajectory.png")

# Figura 2: Tags estimadas vs reais
plt.figure(figsize=(10, 8))
ax2 = plt.gca()
ax2.scatter(est_tag_pos_plot_aligned[:, 0], est_tag_pos_plot_aligned[:, 1], c='red', marker='x', s=120, zorder=5, label='Tags Estimadas (Alinhadas)')
if gt_tags_map:
    ax2.scatter(gt_tags_for_plot[:, 0], gt_tags_for_plot[:, 1], facecolors='none', edgecolors='orange', s=200, linewidth=2.5, zorder=4, label='Tags Reais (Alinhadas)')
ax2.set_title("Comparação de Tags Estimadas vs Reais")
ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
ax2.legend(loc='best'); ax2.grid(True, linestyle='--'); ax2.set_aspect('equal', 'box')
plt.tight_layout()
plt.savefig("definitive_tags.png")

# Apenas mostre se backend suportar; evita warning em headless
if plt.get_backend().lower() not in ("agg", "cairoagg", "svg"):
    plt.show()
print("Gráficos salvos como 'definitive_trajectory.png' e 'definitive_tags.png'")
