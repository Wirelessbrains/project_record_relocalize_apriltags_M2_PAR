import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
import sys
import time

# ==========================================
# CONFIGURAÇÃO DA ÂNCORA
# ==========================================
# Definimos a tag exata que será a origem e os pontos de Start/End
TARGET_ANCHOR_NAME = "tag36h11:0" 

print(f"Configuração: Âncora definida como '{TARGET_ANCHOR_NAME}'")

# ==========================================
# 1. SETUP E CARREGAMENTO DE DADOS
# ==========================================
print("Carregando dataset de medições...")
try:
    filename = 'trajectory_circle_dataset.yaml'
    with open(filename, 'r') as f:
        dataset = yaml.safe_load(f)
except Exception as e:
    print(f"Erro ao abrir arquivo '{filename}': {e}")
    sys.exit(1)

frames = dataset['frames']
num_frames = len(frames)

ground_truth_traj = [] 
tag_id_map = {} 
next_tag_idx = 0
reverse_tag_map = {} 

obs_cam_idxs = []
obs_tag_idxs = []
obs_measurements = []

gt_rel_pos = [] 
gt_rel_rot = [] 

# Variáveis para inicialização
anchor_observation_data = None
frame_idx_where_anchor_first_seen = -1

print(f"Processando {num_frames} frames...")

for i, frame in enumerate(frames):
    # GT Absoluto
    gt_p = frame['ground_truth_pose']['position']
    gt_q = frame['ground_truth_pose']['orientation']
    P_c_gt = np.array([gt_p['x'], gt_p['y'], gt_p['z']])
    R_c_gt = R.from_quat([gt_q['x'], gt_q['y'], gt_q['z'], gt_q['w']])
    ground_truth_traj.append(P_c_gt)

    # Extrair Observações
    if frame.get('detections'):
        for tag_name, pose_data in frame['detections'].items():
            if tag_name not in tag_id_map:
                tag_id_map[tag_name] = next_tag_idx
                reverse_tag_map[next_tag_idx] = tag_name
                next_tag_idx += 1
                
            meas_local = np.array([pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z']])

            # Captura a primeira vez que a ancora aparece para inicializar o otimizador
            if tag_name == TARGET_ANCHOR_NAME and anchor_observation_data is None:
                anchor_observation_data = {
                    'cam_gt_rot': R_c_gt,
                    'meas_local': meas_local,
                    'tag_name': tag_name
                }
                frame_idx_where_anchor_first_seen = i
            
            obs_cam_idxs.append(i)
            obs_tag_idxs.append(tag_id_map[tag_name])
            obs_measurements.append(meas_local)
    
    # Preparar Movimento Relativo
    if i < num_frames - 1:
        curr = frames[i]['ground_truth_pose']
        nxt  = frames[i+1]['ground_truth_pose']
        
        R_c = R.from_quat([curr['orientation']['x'], curr['orientation']['y'], curr['orientation']['z'], curr['orientation']['w']])
        P_c = np.array([curr['position']['x'], curr['position']['y'], curr['position']['z']])
        R_n = R.from_quat([nxt['orientation']['x'], nxt['orientation']['y'], nxt['orientation']['z'], nxt['orientation']['w']])
        
        delta_P = R_c.inv().apply(np.array([nxt['position']['x'], nxt['position']['y'], nxt['position']['z']]) - P_c)
        delta_R = (R_c.inv() * R_n).as_rotvec()
        
        gt_rel_pos.append(delta_P)
        gt_rel_rot.append(delta_R)

# Converter para NumPy
obs_cam_idxs = np.array(obs_cam_idxs, dtype=np.int32)
obs_tag_idxs = np.array(obs_tag_idxs, dtype=np.int32)
obs_measurements = np.array(obs_measurements, dtype=np.float64)
ground_truth_traj = np.array(ground_truth_traj)
gt_rel_pos = np.array(gt_rel_pos)
gt_rel_rot = np.array(gt_rel_rot)

num_tags = len(tag_id_map)
num_observations = len(obs_cam_idxs)
offset_cam_rot = num_frames * 3

# ==========================================
# 3. CONFIGURAÇÃO DE PESOS
# ==========================================
WEIGHT_ANCHOR    = 1000.0  
WEIGHT_OBS       = 20.0     
WEIGHT_SMOOTH_P  = 80.0     
WEIGHT_SMOOTH_R  = 50.0     

# ==========================================
# 4. FUNÇÕES DE CUSTO
# ==========================================
def fun_tags_only(x_tags, c_pos_fixed, c_rot_fixed):
    t_pos = x_tags.reshape(num_tags, 3)
    batch_c_pos = c_pos_fixed[obs_cam_idxs]
    batch_t_pos = t_pos[obs_tag_idxs]
    batch_R_cam = R.from_rotvec(c_rot_fixed[obs_cam_idxs])
    P_diff = batch_t_pos - batch_c_pos
    P_local_est = batch_R_cam.inv().apply(P_diff)
    res_obs = (P_local_est - obs_measurements).ravel() * WEIGHT_OBS
    return res_obs

def fun_traj_smooth(x_traj, t_pos_fixed):
    c_pos = x_traj[:offset_cam_rot].reshape(num_frames, 3)
    c_rot = x_traj[offset_cam_rot:].reshape(num_frames, 3)

    # Âncora (usando idx_start detectado dinamicamente)
    res_anchor_pos = (c_pos[idx_start] - start_pos_vec) * WEIGHT_ANCHOR
    res_anchor_rot = (c_rot[idx_start] - r_vec_start) * WEIGHT_ANCHOR
    
    # Observações
    batch_c_pos = c_pos[obs_cam_idxs]
    batch_t_pos = t_pos_fixed[obs_tag_idxs]
    batch_R_cam = R.from_rotvec(c_rot[obs_cam_idxs])
    P_diff = batch_t_pos - batch_c_pos
    P_local_est = batch_R_cam.inv().apply(P_diff)
    res_obs = (P_local_est - obs_measurements).ravel() * WEIGHT_OBS
    
    # Suavização
    curr_pos = c_pos[:-1]; next_pos = c_pos[1:]
    R_curr = R.from_rotvec(c_rot[:-1])
    R_next = R.from_rotvec(c_rot[1:])
    pred_next_pos = curr_pos + R_curr.apply(gt_rel_pos)
    res_smooth_pos = (next_pos - pred_next_pos).ravel() * WEIGHT_SMOOTH_P

    R_rel_gt = R.from_rotvec(gt_rel_rot) 
    R_next_pred = R_curr * R_rel_gt
    R_diff = R_next_pred.inv() * R_next
    res_smooth_rot = R_diff.as_rotvec().ravel() * WEIGHT_SMOOTH_R

    return np.concatenate([res_anchor_pos, res_anchor_rot, res_obs, res_smooth_pos, res_smooth_rot])

# ==========================================
# 5. INICIALIZAÇÃO (Via Tag Âncora)
# ==========================================
if anchor_observation_data is None:
    print(f"ERRO CRÍTICO: Tag '{TARGET_ANCHOR_NAME}' não encontrada no YAML.")
    print(f"Tags disponíveis: {list(tag_id_map.keys())}")
    sys.exit(1)

P_tag_anchor_world = np.zeros(3)
R_cam_anchor = anchor_observation_data['cam_gt_rot']
P_tag_local = anchor_observation_data['meas_local']
start_pos_vec = P_tag_anchor_world - R_cam_anchor.apply(P_tag_local)
r_vec_start = R_cam_anchor.as_rotvec()
idx_start = frame_idx_where_anchor_first_seen

x0_cam_pos = np.zeros((num_frames, 3))
x0_cam_rot = np.zeros((num_frames, 3))
x0_tag_pos = np.zeros((num_tags, 3))

x0_cam_pos[idx_start] = start_pos_vec
x0_cam_rot[idx_start] = r_vec_start

for i in range(idx_start - 1, -1, -1):
    R_nxt_est = R.from_rotvec(x0_cam_rot[i+1])
    d_pos = gt_rel_pos[i]; d_rot = gt_rel_rot[i]
    R_inc = R.from_rotvec(d_rot)
    R_curr = R_nxt_est * R_inc.inv()
    x0_cam_pos[i] = x0_cam_pos[i+1] - R_curr.apply(d_pos)
    x0_cam_rot[i] = R_curr.as_rotvec()

for i in range(idx_start + 1, num_frames):
    R_prev_est = R.from_rotvec(x0_cam_rot[i-1])
    d_pos = gt_rel_pos[i-1]; d_rot = gt_rel_rot[i-1]
    R_inc = R.from_rotvec(d_rot)
    x0_cam_pos[i] = x0_cam_pos[i-1] + R_prev_est.apply(d_pos)
    x0_cam_rot[i] = (R_prev_est * R_inc).as_rotvec()

for i in range(num_observations):
    t_idx = obs_tag_idxs[i]; c_idx = obs_cam_idxs[i]
    t_name = reverse_tag_map.get(t_idx)
    if t_name == TARGET_ANCHOR_NAME:
        x0_tag_pos[t_idx] = P_tag_anchor_world 
    else:
        R_cam_est = R.from_rotvec(x0_cam_rot[c_idx])
        tag_pos_world = x0_cam_pos[c_idx] + R_cam_est.apply(obs_measurements[i])
        if np.linalg.norm(x0_tag_pos[t_idx]) == 0:
            x0_tag_pos[t_idx] = tag_pos_world
             
x0_raw_cam_pos = x0_cam_pos
x0_raw_cam_rot = x0_cam_rot

# ==========================================
# 6. OTIMIZAÇÃO
# ==========================================
print("\n[PASSO 1/2] Otimizando Tags...")
x0_tags = x0_tag_pos.ravel()
sparsity_step1 = lil_matrix((num_observations * 3, num_tags * 3), dtype=int)
row_idx = 0
for i in range(num_observations):
    sparsity_step1[row_idx:row_idx+3, obs_tag_idxs[i]*3 : (obs_tag_idxs[i]+1)*3] = 1
    row_idx += 3

result_step1 = least_squares(fun_tags_only, x0_tags, jac_sparsity=sparsity_step1, verbose=0, method='trf', x_scale='jac', args=(x0_raw_cam_pos, x0_raw_cam_rot))
est_tag_pos = result_step1.x.reshape(num_tags, 3)

print("\n[PASSO 2/2] Otimizando Trajetória...")
x0_traj = np.hstack([x0_raw_cam_pos.ravel(), x0_raw_cam_rot.ravel()])
n_smooth_res = (num_frames - 1) * 6 
m_step2 = 6 + num_observations * 3 + n_smooth_res 
n_step2 = num_frames * 6 
sparsity_step2 = lil_matrix((m_step2, n_step2), dtype=int)
row_idx = 0
sparsity_step2[0:6, idx_start*3 : (idx_start+1)*3] = 1
sparsity_step2[0:6, num_frames*3 + idx_start*3 : num_frames*3 + (idx_start+1)*3] = 1
row_idx += 6
for i in range(num_observations):
    c_idx = obs_cam_idxs[i]
    sparsity_step2[row_idx:row_idx+3, c_idx*3 : (c_idx+1)*3] = 1
    sparsity_step2[row_idx:row_idx+3, num_frames*3 + c_idx*3 : num_frames*3 + (c_idx+1)*3] = 1
    row_idx += 3
for i in range(num_frames - 1):
    idx_pos_i = i * 3; idx_pos_next = (i + 1) * 3
    idx_rot_i = num_frames * 3 + i * 3; idx_rot_next = num_frames * 3 + (i + 1) * 3
    sparsity_step2[row_idx:row_idx+3, idx_pos_i:idx_pos_i+3] = 1        
    sparsity_step2[row_idx:row_idx+3, idx_pos_next:idx_pos_next+3] = 1 
    sparsity_step2[row_idx:row_idx+3, idx_rot_i:idx_rot_i+3] = 1        
    row_idx += 3
    sparsity_step2[row_idx:row_idx+3, idx_rot_i:idx_rot_i+3] = 1        
    sparsity_step2[row_idx:row_idx+3, idx_rot_next:idx_rot_next+3] = 1 
    row_idx += 3

result_step2 = least_squares(fun_traj_smooth, x0_traj, jac_sparsity=sparsity_step2, verbose=0, method='trf', ftol=1e-12, xtol=1e-12, x_scale='jac', args=(est_tag_pos,))
est_cam_pos_opt = result_step2.x[:offset_cam_rot].reshape(num_frames, 3)
est_cam_rot_opt = result_step2.x[offset_cam_rot:].reshape(num_frames, 3)

# ==========================================
# 7. ALINHAMENTO (Para cálculo de erro)
# ==========================================
print("\nRealizando Alinhamento (Procrustes)...")
gt_tags_file = 'circle_tags_scenario.yaml'
gt_tags_map_align = {}
try:
    with open(gt_tags_file, 'r') as f: gt_scenario_data = yaml.safe_load(f)
    tags_list = [{'tag_name': k, 'position': v['position']} for k, v in gt_scenario_data['tags'].items()] if 'tags' in gt_scenario_data and isinstance(gt_scenario_data['tags'], dict) else gt_scenario_data['tags']
    for t in tags_list: gt_tags_map_align[t['tag_name']] = np.array([t['position']['x'], t['position']['y'], t['position']['z']])
except: pass

src_pts = []; dst_pts = []
for i in range(num_tags):
    t_name = reverse_tag_map.get(i)
    if t_name in gt_tags_map_align:
        src_pts.append(est_tag_pos[i]); dst_pts.append(gt_tags_map_align[t_name])
src_pts = np.array(src_pts); dst_pts = np.array(dst_pts)

if len(src_pts) >= 3:
    source_centroid = np.mean(src_pts, axis=0); target_centroid = np.mean(dst_pts, axis=0)
    H = (src_pts - source_centroid).T @ (dst_pts - target_centroid)
    U, S, Vt = np.linalg.svd(H)
    R_align = Vt.T @ U.T
    if np.linalg.det(R_align) < 0: Vt[2,:] *= -1; R_align = Vt.T @ U.T
    t_align = target_centroid - R_align @ source_centroid
else: R_align, t_align = np.eye(3), np.zeros(3)

est_tag_pos_aligned = (R_align @ est_tag_pos.T).T + t_align
aligned_cam_pos = (R_align @ est_cam_pos_opt.T).T + t_align
diff_aligned = aligned_cam_pos - ground_truth_traj
rmse_aligned = np.sqrt(np.mean(np.linalg.norm(diff_aligned, axis=1)**2))

print(f"RMSE Final: {rmse_aligned:.4f} m")

# ==========================================
# 8. POSES PROJETADAS
# ==========================================
projected_poses = []
R_align_obj = R.from_matrix(R_align) 
for i in range(num_observations):
    c_idx = obs_cam_idxs[i]; t_idx = obs_tag_idxs[i]
    R_cam_aligned = R_align_obj * R.from_rotvec(est_cam_rot_opt[c_idx])
    projected_poses.append(est_tag_pos_aligned[t_idx] - R_cam_aligned.apply(obs_measurements[i]))
projected_poses = np.array(projected_poses)

# ==========================================
# 9. PLOTAGEM (START/END EXATO PARA tag36h11:0)
# ==========================================
start_frame_idx = None
end_frame_idx = None

print(f"\nIdentificando Start/End para {TARGET_ANCHOR_NAME}...")

# Varredura linear do índice 0 até o final
for i, frame in enumerate(frames):
    if frame.get('detections') and TARGET_ANCHOR_NAME in frame['detections']:
        # O primeiro que aparecer é o START
        if start_frame_idx is None:
            start_frame_idx = i
            print(f" -> START definido no Frame {i} (Primeira aparição)")
        
        # O último que aparecer será o END (atualiza sempre)
        end_frame_idx = i

if start_frame_idx is None:
    print(f"AVISO: {TARGET_ANCHOR_NAME} nunca encontrada. Usando fallback 0 e -1.")
    start_frame_idx = 0
    end_frame_idx = num_frames - 1
else:
    print(f" -> END definido no Frame {end_frame_idx} (Última aparição)")

plt.figure(figsize=(10, 8))

# Trajetória
plt.plot(ground_truth_traj[:, 0], ground_truth_traj[:, 1], 'k:', linewidth=1.5, alpha=0.6, label='Ground Truth')
plt.plot(aligned_cam_pos[:, 0], aligned_cam_pos[:, 1], 'b-', linewidth=2.5, label='SLAM Trajectory')

if len(projected_poses) > 0:
    plt.scatter(projected_poses[:, 0], projected_poses[:, 1], c='green', marker='^', s=30, alpha=0.2, label='Camera Poses')

plt.scatter(est_tag_pos_aligned[:, 0], est_tag_pos_aligned[:, 1], c='red', marker='x', s=100, zorder=5, label='Tags')

# START / END Markers
start_pt = aligned_cam_pos[start_frame_idx]
end_pt = aligned_cam_pos[end_frame_idx]

# Start (Verde)
plt.scatter(start_pt[0], start_pt[1], c='lime', s=250, marker='o', edgecolors='black', linewidth=2, zorder=10, label=f'START ({TARGET_ANCHOR_NAME})')
plt.text(start_pt[0], start_pt[1], "  START", fontsize=11, fontweight='bold', color='green')

# End (Vermelho)
plt.scatter(end_pt[0], end_pt[1], c='red', s=250, marker='s', edgecolors='black', linewidth=2, zorder=10, label=f'END ({TARGET_ANCHOR_NAME})')
plt.text(end_pt[0], end_pt[1], "  END", fontsize=11, fontweight='bold', color='darkred')

for i in range(num_tags):
    tag_name = reverse_tag_map.get(i, f"ID_{i}")
    pos = est_tag_pos_aligned[i]
    plt.text(pos[0] + 0.05, pos[1] + 0.05, tag_name, fontsize=8, color='darkred', weight='bold')

plt.title(f"Visual SLAM - Anchor: {TARGET_ANCHOR_NAME}\n(Strict Start/End Detection)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend(loc='best')
plt.grid(True)
plt.axis('equal') 
plt.tight_layout()
plt.savefig('slam_final_graph.png', dpi=300, bbox_inches='tight')
print("Plot salvo.")

# ==========================================
# 10. YAML EXPORT
# ==========================================
print("\nSalvando YAML...")
yaml_output = {'map_tags': [], 'frames': []}
for i in range(num_tags):
    yaml_output['map_tags'].append({'tag_name': reverse_tag_map.get(i), 'position': {'x': float(est_tag_pos_aligned[i][0]), 'y': float(est_tag_pos_aligned[i][1]), 'z': float(est_tag_pos_aligned[i][2])}})
for i in range(num_frames):
    pos = aligned_cam_pos[i]
    qx, qy, qz, qw = (R_align_obj * R.from_rotvec(est_cam_rot_opt[i])).as_quat()
    yaml_output['frames'].append({'timestamp': frames[i].get('timestamp', 0.0), 'pose': {'position': {'x': float(pos[0]), 'y': float(pos[1]), 'z': float(pos[2])}, 'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}}})
with open('dataset_otimizado_final.yaml', 'w') as f: yaml.dump(yaml_output, f, default_flow_style=False, sort_keys=False)

# ==========================================
# 11. ERROR STATS
# ==========================================
print("-" * 75)
errors = []
for i in range(num_tags):
    t_name = reverse_tag_map.get(i)
    if t_name in gt_tags_map_align:
        err = np.linalg.norm(est_tag_pos_aligned[i] - gt_tags_map_align[t_name])
        errors.append(err)
        print(f"{t_name:<15} | Error: {err:.4f} m")

if errors:
    print("-" * 75)
    print(f"Min: {np.min(errors):.4f} m | Max: {np.max(errors):.4f} m | Avg: {np.mean(errors):.4f} m")
plt.show()
