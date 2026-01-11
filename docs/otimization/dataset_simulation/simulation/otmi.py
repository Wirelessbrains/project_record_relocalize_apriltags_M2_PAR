import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
import sys
import time

# ==========================================
# 1. DADOS DE VERDADE (GT TAGS)
# ==========================================
gt_tags_data = {
    "tag36h11:0": {'x': 4.0, 'y': 1.5, 'z': 0.5},
    "tag36h11:2": {'x': 8.0, 'y': -1.5, 'z': 0.5},
    "tag36h11:1": {'x': 6.0, 'y': 1.0, 'z': 0.5},
    "tag36h11:3": {'x': 14.0, 'y': -1.0, 'z': 0.5},
    "tag36h11:4": {'x': 20.0, 'y': 0.0, 'z': 0.5}
}

# ==========================================
# 2. CARREGAMENTO E PREPARAÇÃO
# ==========================================
print("Carregando dataset...")
try:
    with open('s_curve_dataset_large_noisy2', 'r') as f:
        dataset = yaml.safe_load(f)
except Exception as e:
    print(f"Erro ao abrir arquivo: Certifique-se que 's_curve_dataset.yaml' existe. {e}")
    sys.exit(1)

ground_truth_traj = [] 
tag_id_map = {} 
next_tag_idx = 0
reverse_tag_map = {} 

obs_cam_idxs = []
obs_tag_idxs = []
obs_measurements = []

gt_rel_pos = [] 
gt_rel_rot = [] 

frames = dataset['frames']
num_frames = len(frames)

for i, frame in enumerate(frames):
    # GT Absoluto
    gt_p = frame['ground_truth_pose']['position']
    ground_truth_traj.append([gt_p['x'], gt_p['y'], gt_p['z']])

    # Extrair Observações
    if frame.get('detections'):
        for tag_name, pose_data in frame['detections'].items():
            if tag_name not in tag_id_map:
                tag_id_map[tag_name] = next_tag_idx
                reverse_tag_map[next_tag_idx] = tag_name
                next_tag_idx += 1
            
            obs_cam_idxs.append(i)
            obs_tag_idxs.append(tag_id_map[tag_name])
            obs_measurements.append([
                pose_data['position']['x'], 
                pose_data['position']['y'], 
                pose_data['position']['z']
            ])
    
    # Prepara dados para o "Raw" (Dead Reckoning / Odometria)
    if i < num_frames - 1:
        curr = frames[i]['ground_truth_pose']
        nxt  = frames[i+1]['ground_truth_pose']
        
        R_c = R.from_quat([curr['orientation']['x'], curr['orientation']['y'], curr['orientation']['z'], curr['orientation']['w']])
        P_c = np.array([curr['position']['x'], curr['position']['y'], curr['position']['z']])
        
        R_n = R.from_quat([nxt['orientation']['x'], nxt['orientation']['y'], nxt['orientation']['z'], nxt['orientation']['w']])
        P_n = np.array([nxt['position']['x'], nxt['position']['y'], nxt['position']['z']])
        
        delta_P = R_c.inv().apply(P_n - P_c)
        delta_R = (R_c.inv() * R_n).as_rotvec()
        
        gt_rel_pos.append(delta_P)
        gt_rel_rot.append(delta_R)

# Converte para NumPy
obs_cam_idxs = np.array(obs_cam_idxs, dtype=np.int32)
obs_tag_idxs = np.array(obs_tag_idxs, dtype=np.int32)
obs_measurements = np.array(obs_measurements, dtype=np.float64)
ground_truth_traj = np.array(ground_truth_traj)
gt_rel_pos = np.array(gt_rel_pos)
gt_rel_rot = np.array(gt_rel_rot)

num_tags = len(tag_id_map)
num_observations = len(obs_cam_idxs)

offset_cam_rot = num_frames * 3
offset_tag_pos = offset_cam_rot + num_frames * 3
offset_tag_rot = offset_tag_pos + num_tags * 3

# ==========================================
# 3. CONFIGURAÇÃO DE PESOS
# ==========================================
WEIGHT_ANCHOR = 1000.0    
WEIGHT_OBS    = 20.0      

# ==========================================
# 4. MATRIZ DE ESPARSIDADE
# ==========================================
def compute_sparsity():
    m = 6 + num_observations * 3
    n = num_frames * 6 + num_tags * 3
    A = lil_matrix((m, n), dtype=int)
    row_idx = 0
    
    # Âncora
    A[0:6, 0:3] = 1 
    A[0:6, offset_cam_rot : offset_cam_rot+3] = 1 
    row_idx += 6
    
    # Observações
    for i in range(num_observations):
        c_idx = obs_cam_idxs[i]
        t_idx = obs_tag_idxs[i]
        
        A[row_idx:row_idx+3, c_idx*3 : (c_idx+1)*3] = 1
        A[row_idx:row_idx+3, offset_cam_rot + c_idx*3 : offset_cam_rot + (c_idx+1)*3] = 1
        A[row_idx:row_idx+3, offset_tag_pos + t_idx*3 : offset_tag_pos + (t_idx+1)*3] = 1
        row_idx += 3
        
    return A

# ==========================================
# 5. FUNÇÃO DE CUSTO
# ==========================================
def fun_residuals(x):
    c_pos = x[:offset_cam_rot].reshape(num_frames, 3)
    c_rot = x[offset_cam_rot:offset_tag_pos].reshape(num_frames, 3)
    t_pos = x[offset_tag_pos:offset_tag_rot].reshape(num_tags, 3)
    
    # Âncora
    res_anchor_pos = (c_pos[0] - start_pos_vec) * WEIGHT_ANCHOR
    res_anchor_rot = (c_rot[0] - r_vec_start) * WEIGHT_ANCHOR
    
    # Observações
    batch_c_pos = c_pos[obs_cam_idxs]
    batch_t_pos = t_pos[obs_tag_idxs]
    batch_R_cam = R.from_rotvec(c_rot[obs_cam_idxs])
    
    P_diff = batch_t_pos - batch_c_pos
    P_local_est = batch_R_cam.inv().apply(P_diff)
    
    res_obs = (P_local_est - obs_measurements).ravel() * WEIGHT_OBS
    
    return np.concatenate([res_anchor_pos, res_anchor_rot, res_obs])

# ==========================================
# 6. INICIALIZAÇÃO (RAW / DEAD RECKONING)
# ==========================================
print("Gerando estimativa inicial (Raw Path)...")

first_gt = dataset['frames'][0]['ground_truth_pose']
start_pos_vec = np.array([first_gt['position']['x'], first_gt['position']['y'], first_gt['position']['z']])
q = first_gt['orientation']
r_start_obj = R.from_quat([q['x'], q['y'], q['z'], q['w']])
r_vec_start = r_start_obj.as_rotvec()

x0_cam_pos = np.zeros((num_frames, 3))
x0_cam_rot = np.zeros((num_frames, 3))
x0_tag_pos = np.zeros((num_tags, 3))

x0_cam_pos[0] = start_pos_vec
x0_cam_rot[0] = r_vec_start

for i in range(1, num_frames):
    R_prev_est = R.from_rotvec(x0_cam_rot[i-1])
    d_pos = gt_rel_pos[i-1]
    d_rot = gt_rel_rot[i-1]
    R_inc = R.from_rotvec(d_rot)
    
    x0_cam_pos[i] = x0_cam_pos[i-1] + R_prev_est.apply(d_pos)
    x0_cam_rot[i] = (R_prev_est * R_inc).as_rotvec()

# Inicializa Tags (Aproximação inicial baseada no Raw)
tags_initialized_mask = {}
for i in range(num_observations):
    t_idx = obs_tag_idxs[i]
    c_idx = obs_cam_idxs[i]
    
    if t_idx not in tags_initialized_mask:
        cam_pos_est = x0_cam_pos[c_idx]
        cam_rot_vec_est = x0_cam_rot[c_idx]
        R_cam_est = R.from_rotvec(cam_rot_vec_est)
        meas_local = obs_measurements[i]
        
        tag_pos_world = cam_pos_est + R_cam_est.apply(meas_local)
        x0_tag_pos[t_idx] = tag_pos_world
        tags_initialized_mask[t_idx] = True

x0 = np.hstack([x0_cam_pos.ravel(), x0_cam_rot.ravel(), x0_tag_pos.ravel()])

# ==========================================
# 7. OTIMIZAÇÃO
# ==========================================
print("Calculando matriz de esparsidade...")
sparsity = compute_sparsity()

print(f"Iniciando otimização...")
start_time = time.time()

result = least_squares(
    fun_residuals, 
    x0, 
    jac_sparsity=sparsity, 
    verbose=2, 
    method='trf', 
    ftol=1e-12, 
    xtol=1e-12,
    gtol=1e-12,
    x_scale='jac'
)

end_time = time.time()
print(f"Tempo total: {end_time - start_time:.2f}s | Custo Final: {result.cost:.4f}")

x_opt = result.x
est_cam_pos = x_opt[:offset_cam_rot].reshape(num_frames, 3)
est_tag_pos = x_opt[offset_tag_pos:offset_tag_rot].reshape(num_tags, 3)

# ==========================================
# 8. CÁLCULO DA POSE VIA TAGS (Projeção Inversa)
# ==========================================
# Calcula onde o robô estaria se confiássemos 100% na medição da tag e na posição conhecida dela
robot_pos_from_tags = []

for i in range(num_observations):
    c_idx = obs_cam_idxs[i]
    t_idx = obs_tag_idxs[i]
    meas_local = obs_measurements[i]
    
    tag_name = reverse_tag_map[t_idx]
    if tag_name in gt_tags_data:
        gt_t = gt_tags_data[tag_name]
        P_tag_world = np.array([gt_t['x'], gt_t['y'], gt_t['z']])
        R_cam_current = R.from_rotvec(x0_cam_rot[c_idx])
        P_cam_derived = P_tag_world - R_cam_current.apply(meas_local)
        robot_pos_from_tags.append(P_cam_derived)

robot_pos_from_tags = np.array(robot_pos_from_tags)

# ==========================================
# 9. CÁLCULO DE ERROS E RELATÓRIO NO TERMINAL
# ==========================================

# ---------------------------------------------------------
# 9.1. Erro da Trajetória (ATE - Absolute Trajectory Error)
# ---------------------------------------------------------
diff_raw = x0_cam_pos - ground_truth_traj
diff_opt = est_cam_pos - ground_truth_traj

err_raw_per_frame = np.linalg.norm(diff_raw, axis=1)
err_opt_per_frame = np.linalg.norm(diff_opt, axis=1)

rmse_raw = np.sqrt(np.mean(err_raw_per_frame**2))
rmse_opt = np.sqrt(np.mean(err_opt_per_frame**2))

max_err_opt = np.max(err_opt_per_frame)

print("\n" + "="*60)
print(f"{'RELATÓRIO DE PRECISÃO':^60}")
print("="*60)

print("\n[1] TRAJETÓRIA DO ROBÔ (ATE)")
print("-" * 60)
print(f"{'Métrica':<25} | {'Odometria (Raw)':<15} | {'Otimizado':<15}")
print("-" * 60)
print(f"{'RMSE (Erro Médio Global)':<25} | {rmse_raw:.4f} m        | {rmse_opt:.4f} m")
print(f"{'Erro Máximo':<25} | {np.max(err_raw_per_frame):.4f} m        | {max_err_opt:.4f} m")
print("-" * 60)
if rmse_raw > 0:
    improve = (rmse_raw - rmse_opt) / rmse_raw * 100
    print(f"Melhoria na precisão global: {improve:.2f}%")


# ---------------------------------------------------------
# 9.2. Erro de Posicionamento das Tags
# ---------------------------------------------------------
print("\n\n[2] POSICIONAMENTO DAS TAGS (LANDMARKS)")
print("-" * 85)
print(f"{'Tag ID':<12} | {'Erro (m)':<10} | {'Est. (X, Y, Z)':<28} | {'GT (X, Y, Z)':<20}")
print("-" * 85)

tag_errors = []

for i in range(num_tags):
    t_name = reverse_tag_map.get(i, f"ID_{i}")
    pos_est = est_tag_pos[i]
    
    if t_name in gt_tags_data:
        gt_t = gt_tags_data[t_name]
        pos_gt = np.array([gt_t['x'], gt_t['y'], gt_t['z']])
        
        error = np.linalg.norm(pos_est - pos_gt)
        tag_errors.append(error)
        
        str_est = f"({pos_est[0]:6.2f}, {pos_est[1]:6.2f}, {pos_est[2]:5.2f})"
        str_gt  = f"({pos_gt[0]:6.2f}, {pos_gt[1]:6.2f}, {pos_gt[2]:5.2f})"
        
        alert = " <!>" if error > 0.1 else ""
        
        print(f"{t_name:<12} | {error:.4f}{alert:<5} | {str_est:<28} | {str_gt}")
    else:
        print(f"{t_name:<12} | {'--':<10} | {str(pos_est):<28} | Sem GT")

print("-" * 85)
if tag_errors:
    print(f"Erro Médio das Tags: {np.mean(tag_errors):.4f} m")
print("="*60 + "\n")


# ==========================================
# 10. PLOTAGEM (MAPA XY)
# ==========================================
plt.figure(figsize=(10, 8))

# 1. Raw / Dead Reckoning (Cinza Tracejado)
plt.plot(x0_cam_pos[:, 0], x0_cam_pos[:, 1], color='gray', linestyle='--', linewidth=2, alpha=0.6, label='Raw Odometry (Dead Reckoning)')

# 2. GT Trajetória (Preto Pontilhado)
plt.plot(ground_truth_traj[:, 0], ground_truth_traj[:, 1], 'k:', linewidth=1.5, alpha=0.7, label='Ground Truth Traj')

# 3. Poses Derivadas das Tags (Pontos Verdes) - Onde as tags dizem que o robô está
if len(robot_pos_from_tags) > 0:
    plt.scatter(robot_pos_from_tags[:, 0], robot_pos_from_tags[:, 1], 
                c='green', marker='^', s=40, alpha=0.8, label='Pose Robô via Tag (Medição)')

# 4. Otimizado (Azul Contínuo)
plt.plot(est_cam_pos[:, 0], est_cam_pos[:, 1], 'b-', linewidth=2, label=f'Trajetória Otimizada (RMSE: {rmse_opt:.3f}m)')

# 5. Tags (Vermelho X) e GT (Círculos)
plt.scatter(est_tag_pos[:, 0], est_tag_pos[:, 1], c='red', marker='x', s=100, zorder=5, label='Tags Estimadas (Pós-Opt)')

for t_name, t_data in gt_tags_data.items():
    plt.scatter(t_data['x'], t_data['y'], s=150, facecolors='none', edgecolors='k', linestyle=':', linewidth=1.5, label='Tag Real (GT)' if t_name=='tag36h11:0' else "")

plt.title("Otimização de Trajetória: Odometria vs. Correção por Tags")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend(loc='best')
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()
