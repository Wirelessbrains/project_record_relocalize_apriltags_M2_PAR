import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
import sys
import os

# ==========================================
# 1. SETUP E CARREGAMENTO DE DADOS
# ==========================================
print("Carregando dataset de medições...")
filename = 'circle_dense_truth.yaml' 

try:
    with open(filename, 'r') as f:
        dataset = yaml.safe_load(f)
except Exception as e:
    print(f"Erro ao abrir arquivo '{filename}': {e}")
    sys.exit(1)

# Variáveis Globais
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

first_observation_data = None
first_frame_with_detection = -1

print(f"Total de frames encontrados: {num_frames}")

for i, frame in enumerate(frames):
    # A. Carregar Trajetória de Entrada (Odometria/Pose Inicial)
    curr_p = frame['pose']['position']
    curr_q = frame['pose']['orientation']
    P_c_input = np.array([curr_p['x'], curr_p['y'], curr_p['z']])
    R_c_input = R.from_quat([curr_q['x'], curr_q['y'], curr_q['z'], curr_q['w']])
    ground_truth_traj.append(P_c_input)

    # B. Extrair Observações (Prioridade para 'detections')
    detection_field = frame.get('detections') or frame.get('learned') or frame.get('tags')
    
    if detection_field: 
        if first_frame_with_detection == -1:
             first_frame_with_detection = i
             
        for tag_name, pose_data in detection_field.items():
            if tag_name not in tag_id_map:
                tag_id_map[tag_name] = next_tag_idx
                reverse_tag_map[next_tag_idx] = tag_name
                next_tag_idx += 1
                
            meas_local = np.array([pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z']])

            if i == first_frame_with_detection and first_observation_data is None:
                first_observation_data = {
                    'cam_gt_rot': R_c_input, 
                    'meas_local': meas_local,
                    'tag_name': tag_name
                }
                
            obs_cam_idxs.append(i)
            obs_tag_idxs.append(tag_id_map[tag_name])
            obs_measurements.append(meas_local)
    
    # C. Preparar Movimento Relativo (Apenas para regularização suave)
    if i < num_frames - 1:
        nxt_p = frames[i+1]['pose']['position']
        nxt_q = frames[i+1]['pose']['orientation']
        R_n = R.from_quat([nxt_q['x'], nxt_q['y'], nxt_q['z'], nxt_q['w']])
        
        delta_P = R_c_input.inv().apply(np.array([nxt_p['x'], nxt_p['y'], nxt_p['z']]) - P_c_input)
        delta_R = (R_c_input.inv() * R_n).as_rotvec()
        
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

print(f"Dados carregados. Frames com detecção começam em: {first_frame_with_detection}")
print(f"Tags únicas: {num_tags} | Total de observações: {num_observations}")

# ==========================================
# 2. FUNÇÕES DE CUSTO
# ==========================================
def fun_tags_only(x_tags, c_pos_fixed, c_rot_fixed):
    t_pos = x_tags.reshape(num_tags, 3)
    batch_c_pos = c_pos_fixed[obs_cam_idxs]
    batch_t_pos = t_pos[obs_tag_idxs]
    batch_R_cam = R.from_rotvec(c_rot_fixed[obs_cam_idxs])
    
    # Erro de projeção: Onde a tag deveria estar no frame da câmera vs Onde foi medida
    P_diff = batch_t_pos - batch_c_pos 
    P_local_est = batch_R_cam.inv().apply(P_diff) 
    
    res_obs = (P_local_est - obs_measurements).ravel() * WEIGHT_OBS
    return res_obs

def fun_traj_smooth(x_traj, t_pos_fixed):
    c_pos = x_traj[:offset_cam_rot].reshape(num_frames, 3)
    c_rot = x_traj[offset_cam_rot:].reshape(num_frames, 3)

    # 1. Âncora (Prende o primeiro frame para não derivar o mundo)
    res_anchor_pos = (c_pos[first_frame_with_detection] - start_pos_vec) * WEIGHT_ANCHOR
    res_anchor_rot = (c_rot[first_frame_with_detection] - r_vec_start) * WEIGHT_ANCHOR
    
    # 2. Observações (Erro de Projeção)
    batch_c_pos = c_pos[obs_cam_idxs]
    batch_t_pos = t_pos_fixed[obs_tag_idxs]
    batch_R_cam = R.from_rotvec(c_rot[obs_cam_idxs])
    
    P_diff = batch_t_pos - batch_c_pos
    P_local_est = batch_R_cam.inv().apply(P_diff)
    res_obs = (P_local_est - obs_measurements).ravel() * WEIGHT_OBS
    
    # 3. Suavização (Regularização mínima)
    curr_pos = c_pos[:-1]; next_pos = c_pos[1:]
    R_curr = R.from_rotvec(c_rot[:-1])
    R_next = R.from_rotvec(c_rot[1:])
    R_rel_gt = R.from_rotvec(gt_rel_rot) 
    
    pred_next_pos = curr_pos + R_curr.apply(gt_rel_pos)
    res_smooth_pos = (next_pos - pred_next_pos).ravel() * WEIGHT_SMOOTH_P

    R_next_pred = R_curr * R_rel_gt
    R_diff = R_next_pred.inv() * R_next 
    res_smooth_rot = R_diff.as_rotvec().ravel() * WEIGHT_SMOOTH_R

    return np.concatenate([res_anchor_pos, res_anchor_rot, res_obs, res_smooth_pos, res_smooth_rot])

# ==========================================
# 3. CONFIGURAÇÃO DE PESOS
# ==========================================
WEIGHT_ANCHOR    = 1000.0   
WEIGHT_OBS       = 500.0    
WEIGHT_SMOOTH_P  = 0.001    
WEIGHT_SMOOTH_R  = 0.001    

# ==========================================
# 4. INICIALIZAÇÃO E ÂNCORA
# ==========================================
print("Gerando estimativa inicial...")
if first_observation_data is None:
    print("ERRO CRÍTICO: Nenhuma tag detectada no arquivo.")
    sys.exit(1)

# Definir sistema de coordenadas baseado na primeira detecção
P_tag_anchor_world = np.zeros(3) 
R_cam_anchor = first_observation_data['cam_gt_rot']
P_tag_local = first_observation_data['meas_local']
start_pos_vec = P_tag_anchor_world - R_cam_anchor.apply(P_tag_local) 
r_vec_start = R_cam_anchor.as_rotvec()
idx_start = first_frame_with_detection

x0_cam_pos = np.zeros((num_frames, 3))
x0_cam_rot = np.zeros((num_frames, 3))
x0_tag_pos = np.zeros((num_tags, 3))

# Inicializar câmeras com os dados de entrada
for i in range(num_frames):
    p = frames[i]['pose']['position']
    q = frames[i]['pose']['orientation']
    R_c = R.from_quat([q['x'], q['y'], q['z'], q['w']])
    x0_cam_pos[i] = np.array([p['x'], p['y'], p['z']])
    x0_cam_rot[i] = R_c.as_rotvec()

# Transformar tudo para alinhar com a âncora
T_offset = start_pos_vec - x0_cam_pos[idx_start]
R_offset_obj = R.from_rotvec(r_vec_start) * R.from_rotvec(x0_cam_rot[idx_start]).inv()

for i in range(num_frames):
    x0_cam_pos[i] += T_offset
    R_curr = R.from_rotvec(x0_cam_rot[i])
    R_new = R_offset_obj * R_curr
    x0_cam_rot[i] = R_new.as_rotvec()
    
# Inicializar tags (Média das projeções)
tag_projections = [[] for _ in range(num_tags)]
for i in range(num_observations):
    t_idx = obs_tag_idxs[i]; c_idx = obs_cam_idxs[i]
    R_cam_est = R.from_rotvec(x0_cam_rot[c_idx])
    tag_pos_world = x0_cam_pos[c_idx] + R_cam_est.apply(obs_measurements[i])
    tag_projections[t_idx].append(tag_pos_world)
    
for t_idx in range(num_tags):
    if tag_projections[t_idx]:
        x0_tag_pos[t_idx] = np.median(tag_projections[t_idx], axis=0)
            
# Fixar âncora
tag_anchor_name = first_observation_data['tag_name']
x0_tag_pos[tag_id_map[tag_anchor_name]] = P_tag_anchor_world
            
x0_raw_cam_pos = x0_cam_pos
x0_raw_cam_rot = x0_cam_rot

# ==========================================
# 5. OTIMIZAÇÃO (Solver)
# ==========================================

# --- PASSO 1: Otimizar Tags (Câmeras fixas) ---
print("\n[PASSO 1/2] Refinando Posição das Tags...")
x0_tags = x0_tag_pos.ravel()
sparsity_step1 = lil_matrix((num_observations * 3, num_tags * 3), dtype=int)
row_idx = 0
for i in range(num_observations):
    sparsity_step1[row_idx:row_idx+3, obs_tag_idxs[i]*3 : (obs_tag_idxs[i]+1)*3] = 1
    row_idx += 3

result_step1 = least_squares(
    fun_tags_only, x0_tags, jac_sparsity=sparsity_step1, verbose=0, method='trf', 
    x_scale='jac', args=(x0_raw_cam_pos, x0_raw_cam_rot)
)
est_tag_pos = result_step1.x.reshape(num_tags, 3)

# --- PASSO 2: Bundle Adjustment (Câmeras + Tags) ---
print("[PASSO 2/2] Otimização Global (Trajectory + Map)...")
x0_traj = np.hstack([x0_raw_cam_pos.ravel(), x0_raw_cam_rot.ravel()])

n_smooth_res = (num_frames - 1) * 6  
m_step2 = 6 + num_observations * 3 + n_smooth_res  
n_step2 = num_frames * 6  

sparsity_step2 = lil_matrix((m_step2, n_step2), dtype=int)
row_idx = 0
# Anchor
sparsity_step2[0:6, idx_start*3:(idx_start+1)*3] = 1
sparsity_step2[0:6, num_frames*3+idx_start*3:num_frames*3+(idx_start+1)*3] = 1
row_idx += 6
# Obs
for i in range(num_observations):
    c_idx = obs_cam_idxs[i]
    sparsity_step2[row_idx:row_idx+3, c_idx*3:(c_idx+1)*3] = 1
    sparsity_step2[row_idx:row_idx+3, num_frames*3+c_idx*3:num_frames*3+(c_idx+1)*3] = 1
    row_idx += 3
# Smooth
for i in range(num_frames - 1):
    idx_pos_i = i * 3
    idx_rot_i = num_frames * 3 + i * 3
    sparsity_step2[row_idx:row_idx+6, idx_pos_i:idx_pos_i+6] = 1 
    sparsity_step2[row_idx:row_idx+6, idx_rot_i:idx_rot_i+6] = 1 
    row_idx += 6

result_step2 = least_squares(
    fun_traj_smooth, x0_traj, jac_sparsity=sparsity_step2, verbose=0, method='trf', 
    ftol=1e-12, xtol=1e-12, x_scale='jac', args=(est_tag_pos,)
)

est_cam_pos_opt = result_step2.x[:offset_cam_rot].reshape(num_frames, 3)
est_cam_rot_opt = result_step2.x[offset_cam_rot:].reshape(num_frames, 3)
print(f"Otimização finalizada. Custo Final: {result_step2.cost:.4f}")



# ==========================================
# 6. ALINHAMENTO COM GROUND TRUTH (APENAS REFERENCIAL)
# ==========================================
gt_tags_file = 'circle_tags_scenario_dense.yaml'
gt_tags_map_align = {}
use_alignment = False

if os.path.exists(gt_tags_file):
    try:
        with open(gt_tags_file, 'r') as f:
            gt_data = yaml.safe_load(f)
        
        tags_raw = gt_data.get('tags', gt_data)
        if isinstance(tags_raw, dict):
            iterator = tags_raw.items()
        elif isinstance(tags_raw, list):
            iterator = [(t.get('tag_name'), t.get('position')) for t in tags_raw]
        else:
            iterator = []

        for k, v in iterator:
            pos_data = v if 'x' in v else v.get('position')
            if k and pos_data:
                gt_tags_map_align[k] = np.array([pos_data['x'], pos_data['y'], pos_data['z']])
        
        use_alignment = True
        print(f"\nAlinhamento: Arquivo de Scenario carregado com {len(gt_tags_map_align)} tags.")
    except:
        print("\nAVISO: Falha ao ler arquivo de scenario. Alinhamento GT desativado.")
else:
    print("\nAVISO: Arquivo 'circle_tags_scenario.yaml' não encontrado. Usando referencial local.")

# Calcular Transformação R, t (Procrustes) para alinhar visualmente
# Nota: Isso apenas move os pontos estimados para ficarem "em cima" do grid do scenario,
# mas não altera a forma da nuvem estimada.
src_pts, dst_pts = [], []
if use_alignment:
    for i in range(num_tags):
        name = reverse_tag_map.get(i)
        if name in gt_tags_map_align:
            src_pts.append(est_tag_pos[i])
            dst_pts.append(gt_tags_map_align[name])
    src_pts = np.array(src_pts)
    dst_pts = np.array(dst_pts)

R_align, t_align = np.eye(3), np.zeros(3)
if len(src_pts) >= 3:
    cent_src, cent_dst = np.mean(src_pts, axis=0), np.mean(dst_pts, axis=0)
    H = (src_pts - cent_src).T @ (dst_pts - cent_dst)
    U, S, Vt = np.linalg.svd(H)
    R_align = Vt.T @ U.T
    if np.linalg.det(R_align) < 0:
        Vt[2,:] *= -1
        R_align = Vt.T @ U.T
    t_align = cent_dst - R_align @ cent_src

# Aplicar Alinhamento aos Resultados Otimizados
est_tag_pos_aligned = (R_align @ est_tag_pos.T).T + t_align
aligned_cam_pos = (R_align @ est_cam_pos_opt.T).T + t_align

# ==========================================
# 7. REPROJEÇÃO DA POSE DA CÂMERA (CORRIGIDO)
# ==========================================
# Regra: P_cam_reproj = Tag_Estimada - R_cam_opt * Medição
# IMPORTANTE: Não usar 'gt_tags_map_align' aqui! Usar apenas resultados da otimização.

print("\n" + "="*70)
print(f"{'ANÁLISE DE CONSISTÊNCIA: REPROJEÇÃO INVERSA':^70}")
print("="*70)

camera_obs_points = []
R_align_obj = R.from_matrix(R_align)

for i in range(num_observations):
    c_idx = obs_cam_idxs[i]
    t_idx = obs_tag_idxs[i]
    
    # 1. Obter Pose Otimizada da Câmera (Rotação)
    # Precisamos rotacioná-la para o frame alinhado para o plot bater
    r_vec_opt = est_cam_rot_opt[c_idx]
    R_cam_local = R.from_rotvec(r_vec_opt)
    R_cam_world_aligned = R_align_obj * R_cam_local
    
    # 2. Obter Posição da Tag Estimada (ESTIMADA, NÃO REAL)
    # Estamos usando a versão alinhada apenas para coordenadas baterem no plot
    P_tag_est = est_tag_pos_aligned[t_idx] 
    
    # 3. Medição do Sensor
    P_meas_local = obs_measurements[i]
    
    # 4. Cálculo Inverso: Onde a câmera está baseada SOMENTE nesta estimativa?
    # P_tag = P_cam + R_cam * P_meas
    # P_cam = P_tag - R_cam * P_meas
    P_cam_derived = P_tag_est - R_cam_world_aligned.apply(P_meas_local)
    
    camera_obs_points.append(P_cam_derived)

camera_obs_points = np.array(camera_obs_points)

# Tabela de Erro das Tags (Apenas aqui usamos o Scenario para comparar)
total_error = 0.0
count_match = 0
gt_x_plot, gt_y_plot = [], []

print(f"{'Tag Name':<15} | {'Estimado (x,y,z)':<25} | {'Real (Scenario)':<25} | {'Erro (m)':<10}")
print("-" * 85)

for i in range(num_tags):
    name = reverse_tag_map.get(i)
    est = est_tag_pos_aligned[i]
    line = f"{name:<15} | {est[0]:.2f}, {est[1]:.2f}, {est[2]:.2f}".ljust(43)
    
    if name in gt_tags_map_align:
        gt = gt_tags_map_align[name]
        err = np.linalg.norm(est - gt)
        line += f" | {gt[0]:.2f}, {gt[1]:.2f}, {gt[2]:.2f}".ljust(28) + f" | {err:.4f}"
        total_error += err
        count_match += 1
        gt_x_plot.append(gt[0]); gt_y_plot.append(gt[1])
    else:
        line += " | --- GT Missing ---".ljust(28) + " | ----"
    print(line)

print("-" * 85)
print(f"Erro Médio das Tags: {total_error/max(1, count_match):.4f} m" if count_match else "Sem comparação GT.")

# ==========================================
# 8. PLOTAGEM FINAL
# ==========================================
plt.figure(figsize=(12, 10))

# 1. Trajetória Entrada (Opcional, apenas referência)
if use_alignment:
    # Transformando a entrada também para alinhar visualmente, se quiser
    pass 

# 2. "Nuvem" de Câmeras Reprojetadas (Verdes)
# Representa a consistência das estimativas. Deve formar um túnel ao redor da linha azul.
if len(camera_obs_points) > 0:
    plt.scatter(camera_obs_points[:,0], camera_obs_points[:,1], 
                c='green', 
                s=40,            # Aumentado para melhor visibilidade
                alpha=0.5,     
                marker='^',    
                label='Pose camera')

# 3. Trajetória Otimizada (SLAM) - Linha Azul
plt.plot(aligned_cam_pos[:,0], aligned_cam_pos[:,1], 'b-', linewidth=2, label='Otmisation')

# 4. Tags Estimadas (X Vermelho)
plt.scatter(est_tag_pos_aligned[:,0], est_tag_pos_aligned[:,1], c='red', marker='x', s=120, zorder=5, label='Tags Estime')

# 5. Tags Ground Truth (Círculo Laranja - APENAS VISUALIZAÇÃO)
if count_match > 0:
    plt.scatter(gt_x_plot, gt_y_plot, facecolors='none', edgecolors='orange', s=180, linewidth=2.5, label='Scenario (Ref)')
    # Linhas de erro (cinza)
    for i in range(num_tags):
        name = reverse_tag_map.get(i)
        if name in gt_tags_map_align:
            est, gt = est_tag_pos_aligned[i], gt_tags_map_align[name]
            plt.plot([est[0], gt[0]], [est[1], gt[1]], color='gray', linestyle='--', linewidth=0.8)

# Textos
for i in range(num_tags):
    p = est_tag_pos_aligned[i]
    plt.text(p[0]+0.15, p[1]+0.15, f"{reverse_tag_map[i]}", fontsize=9, color='darkred', fontweight='bold')

plt.title("Visual SLAM: Otimização e Consistência das Observações")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend(loc='upper right')
plt.grid(True, linestyle='--', alpha=0.5)
plt.axis('equal')
plt.tight_layout()

plt.savefig('slam_camera_reprojection.png', dpi=300)
print("Gráfico salvo como 'slam_camera_reprojection.png'.")
plt.show()
