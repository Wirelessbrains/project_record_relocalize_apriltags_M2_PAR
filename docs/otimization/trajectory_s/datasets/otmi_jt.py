import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline
from scipy.sparse import lil_matrix
import sys

# ==========================================
# 1. CARREGAMENTO E DADOS
# ==========================================
print("--- SLAM FINAL: TRIÂNGULOS PEQUENOS (ROBUSTO) ---")

try:
    with open('trajectory_circle_dataset.yaml', 'r') as f:
        dataset = yaml.safe_load(f)
    frames = dataset['frames']
except Exception as e:
    print(f"Erro ao abrir dataset: {e}"); sys.exit(1)

gt_scenario_tags = {}
try:
    with open('circle_tags_scenario.yaml', 'r') as f:
        scen = yaml.safe_load(f)
        tl = scen['tags'] if isinstance(scen['tags'], list) else [{'tag_name':k, 'position':v['position']} for k,v in scen['tags'].items()]
        for t in tl:
            gt_scenario_tags[t['tag_name']] = np.array([t['position']['x'], t['position']['y'], t['position']['z']])
except: pass

obs_data = [] 
keyframes_indices = []
tag_id_map = {}; reverse_tag_map = {}; next_tag_idx = 0
ground_truth_traj_ref = [] 

print("Processando frames...")
for i, frame in enumerate(frames):
    # --- CORREÇÃO AQUI: Verificação segura do GT ---
    if 'ground_truth_pose' in frame:
        p = frame['ground_truth_pose']['position']
        ground_truth_traj_ref.append([p['x'], p['y'], p['z']])
    
    # Processa Detecções
    if frame.get('detections'):
        if not keyframes_indices or keyframes_indices[-1] != i:
            keyframes_indices.append(i)
        kf_idx = len(keyframes_indices) - 1
        
        for name, data in frame['detections'].items():
            if name not in tag_id_map:
                tag_id_map[name] = next_tag_idx
                reverse_tag_map[next_tag_idx] = name
                next_tag_idx += 1
            mp = data['position']; mq = data['orientation']
            obs_data.append({
                'kf_idx': kf_idx, 'tag_idx': tag_id_map[name],
                'p': np.array([mp['x'], mp['y'], mp['z']]),
                'q': np.array([mq['x'], mq['y'], mq['z'], mq['w']])
            })

num_kf = len(keyframes_indices)
num_tags = len(tag_id_map)
num_obs = len(obs_data)
has_gt = len(ground_truth_traj_ref) > 0 # Flag para saber se temos GT

print(f"Keyframes: {num_kf} | Tags: {num_tags} | GT Disponível: {has_gt}")

if num_kf < 2:
    print("ERRO: Poucos keyframes para SLAM.")
    sys.exit(1)

# ==========================================
# 2. LÓGICA DE OTIMIZAÇÃO
# ==========================================
print("Calculando trajetória...")
# Init
x_kf_pos = np.zeros((num_kf, 3)); x_kf_rot = np.zeros((num_kf, 3))
x_tag_pos = np.zeros((num_tags, 3)); x_tag_rot = np.zeros((num_tags, 3))
radius = 3.0
for k in range(num_kf):
    ang = 2 * np.pi * k / num_kf
    x_kf_pos[k] = [radius * np.cos(ang), radius * np.sin(ang), 0]
    x_kf_rot[k] = R.from_euler('z', ang + np.pi).as_rotvec()

p0 = x_kf_pos[0].copy(); r0_inv = R.from_rotvec(x_kf_rot[0]).inv()
for k in range(num_kf):
    x_kf_pos[k] = r0_inv.apply(x_kf_pos[k] - p0)
    x_kf_rot[k] = (r0_inv * R.from_rotvec(x_kf_rot[k])).as_rotvec()

tag_cnt = np.zeros(num_tags)
for obs in obs_data:
    k = obs['kf_idx']; t = obs['tag_idx']
    R_wc = R.from_rotvec(x_kf_rot[k]); P_wc = x_kf_pos[k]
    P_wt = P_wc + R_wc.apply(obs['p'])
    R_wt = R_wc * R.from_quat(obs['q'])
    if tag_cnt[t] == 0: x_tag_pos[t] = P_wt; x_tag_rot[t] = R_wt.as_rotvec()
    else: x_tag_pos[t] = (x_tag_pos[t]*tag_cnt[t] + P_wt)/(tag_cnt[t]+1)
    tag_cnt[t] += 1

# Solver
obs_kf = np.array([o['kf_idx'] for o in obs_data])
obs_tag = np.array([o['tag_idx'] for o in obs_data])
obs_p = np.array([o['p'] for o in obs_data])
obs_q = np.array([o['q'] for o in obs_data])

def cost_func(x, mode='global', fixed_kf_p=None, fixed_kf_r=None):
    if mode == 'tags':
        kp, kr = fixed_kf_p, fixed_kf_r
        tp, tr = x[:num_tags*3].reshape(num_tags,3), x[num_tags*3:].reshape(num_tags,3)
    else:
        off_r=num_kf*3; off_tp=num_kf*6; off_tr=num_kf*6+num_tags*3
        kp, kr = x[:off_r].reshape(num_kf,3), x[off_r:off_tp].reshape(num_kf,3)
        tp, tr = x[off_tp:off_tr].reshape(num_tags,3), x[off_tr:].reshape(num_tags,3)
    bkp = kp[obs_kf]; bkr = kr[obs_kf]; btp = tp[obs_tag]; btr = tr[obs_tag]
    R_wc = R.from_rotvec(bkr)
    res_p = (R_wc.inv().apply(btp - bkp) - obs_p).ravel()
    res_r = (R.from_quat(obs_q).inv() * (R_wc.inv() * R.from_rotvec(btr))).as_rotvec().ravel() * 2.0
    if mode == 'global': return np.concatenate([res_p, res_r, (kp[0])*1000, (kr[0])*1000])
    return np.concatenate([res_p, res_r])

# Run Optimization
try:
    res1 = least_squares(cost_func, np.hstack([x_tag_pos.ravel(), x_tag_rot.ravel()]), args=('tags', x_kf_pos, x_kf_rot))
    x_tag_pos, x_tag_rot = res1.x[:num_tags*3].reshape(num_tags,3), res1.x[num_tags*3:].reshape(num_tags,3)

    m = num_obs*6 + 6; n = num_kf*6 + num_tags*6
    sparsity = lil_matrix((m, n), dtype=int)
    off_r=num_kf*3; off_tp=num_kf*6; off_tr=num_kf*6+num_tags*3
    for i in range(num_obs):
        r=i*6; c=obs_kf[i]; t=obs_tag[i]
        sparsity[r:r+6, c*3:c*3+3]=1; sparsity[r:r+6, off_r+c*3:off_r+c*3+3]=1
        sparsity[r:r+6, off_tp+t*3:off_tp+t*3+3]=1; sparsity[r:r+6, off_tr+t*3:off_tr+t*3+3]=1
    sparsity[num_obs*6:, 0:6] = 1

    res2 = least_squares(cost_func, np.hstack([x_kf_pos.ravel(), x_kf_rot.ravel(), x_tag_pos.ravel(), x_tag_rot.ravel()]), jac_sparsity=sparsity, method='trf', x_scale='jac')
    opt_kf_pos, opt_kf_rot = res2.x[:off_r].reshape(num_kf,3), res2.x[off_r:off_tp].reshape(num_kf,3)
    opt_tag_pos, opt_tag_rot = res2.x[off_tp:off_tr].reshape(num_tags,3), res2.x[off_tr:].reshape(num_tags,3)
except Exception as e:
    print(f"Erro na otimização: {e}"); sys.exit(1)

# Alinhamento
src_pts, dst_pts = [], []
for i in range(num_tags):
    name = reverse_tag_map[i]
    if name in gt_scenario_tags:
        src_pts.append(opt_tag_pos[i]); dst_pts.append(gt_scenario_tags[name])
src_pts = np.array(src_pts); dst_pts = np.array(dst_pts)

scale = 1.0; t_align = np.zeros(3); R_align = np.eye(3)

if len(src_pts) >= 3:
    mu_s=np.mean(src_pts,0); mu_d=np.mean(dst_pts,0)
    cov=(src_pts-mu_s).T@(dst_pts-mu_d)
    U,S,Vt=np.linalg.svd(cov)
    R_align=Vt.T@U.T
    if np.linalg.det(R_align)<0: Vt[2,:]*=-1; R_align=Vt.T@U.T
    scale = np.sum(S)/np.sum(np.var(src_pts, axis=0)*len(src_pts))
    t_align = mu_d - scale*(R_align@mu_s)
else:
    print("AVISO: Tags insuficientes para alinhamento global. Usando escala 1.0.")

final_kf_pos = scale*(R_align@opt_kf_pos.T).T + t_align
final_tag_pos = scale*(R_align@opt_tag_pos.T).T + t_align
R_obj = R.from_matrix(R_align)
final_kf_rot = [R_obj*R.from_rotvec(r) for r in opt_kf_rot]
final_tag_rot = [R_obj*R.from_rotvec(r) for r in opt_tag_rot]

# ==========================================
# 3. VISUALIZAÇÃO COM TRIÂNGULOS PEQUENOS
# ==========================================
print("Gerando gráfico...")

VIS_STEP = 12       
TRIANGLE_SIZE = 0.15 

# Interpolação
# Se não tiver GT, usamos o número total de frames para definir o eixo de tempo
total_len = len(ground_truth_traj_ref) if has_gt else len(frames)
cs = CubicSpline(keyframes_indices, final_kf_pos)
traj_smooth = cs(np.arange(total_len))

rmse_str = "N/A"
if has_gt:
    rmse = np.sqrt(np.mean(np.linalg.norm(traj_smooth - np.array(ground_truth_traj_ref), axis=1)**2))
    rmse_str = f"{rmse:.2f}m"

plt.figure(figsize=(12, 12))
ax = plt.gca()

# 1. Trajetória GT (Se existir)
if has_gt:
    gt_arr = np.array(ground_truth_traj_ref)
    plt.plot(gt_arr[:,0], gt_arr[:,1], 'k--', alpha=0.3, label='GT')

# 2. Trajetória SLAM
plt.plot(traj_smooth[:,0], traj_smooth[:,1], 'b-', linewidth=2.0, zorder=2) 

# 3. Tags
def draw_axes(p, r, sz=0.3):
    cols = ['r','g','b']
    for i, v in enumerate(np.eye(3)):
        vec = r.apply(v)*sz
        plt.plot([p[0],p[0]+vec[0]], [p[1],p[1]+vec[1]], cols[i], zorder=3)

for i in range(num_tags):
    p = final_tag_pos[i]
    plt.text(p[0]+0.2, p[1], reverse_tag_map[i], fontsize=8, weight='bold', zorder=4)
    draw_axes(p, final_tag_rot[i])

# 4. Triângulos Pequenos + Linhas
def get_triangle_coords(pos, rot, size):
    tip = np.array([0, 0, 1]) * size 
    width = size * 0.5
    back_left = np.array([-width, 0, 0])
    back_right = np.array([width, 0, 0])
    pts = [rot.apply(p) + pos for p in [tip, back_right, back_left]]
    return np.array([[p[0], p[1]] for p in pts])

obs_kf_map = np.array([o['kf_idx'] for o in obs_data])

for i in range(0, num_kf, VIS_STEP):
    p = final_kf_pos[i]
    r = final_kf_rot[i]
    
    # Triângulo
    coords = get_triangle_coords(p, r, size=TRIANGLE_SIZE)
    poly = Polygon(coords, closed=True, facecolor='cyan', edgecolor='#000080', alpha=0.9, linewidth=1, zorder=5)
    ax.add_patch(poly)
    
    # Linhas de Visão
    obs_idxs = np.where(obs_kf_map == i)[0]
    for oid in obs_idxs:
        t_idx = obs_data[oid]['tag_idx']
        p_tag = final_tag_pos[t_idx]
        plt.plot([p[0], p_tag[0]], [p[1], p_tag[1]], color='magenta', linestyle=':', linewidth=0.6, alpha=0.4, zorder=1)

# Legenda
legend_elements = [
    Line2D([0], [0], color='b', lw=2.5, label=f'Trajetória SLAM (RMSE: {rmse_str})'),
    Line2D([0], [0], color='magenta', lw=1, ls=':', alpha=0.8, label='Raio de Visão'),
    Line2D([0], [0], marker='s', color='w', label='AprilTags', markerfacecolor='k', markersize=6),
    Line2D([0], [0], marker='^', color='w', label='Câmera (Pose)', 
           markerfacecolor='cyan', markeredgecolor='#000080', markersize=8)
]
if has_gt:
    legend_elements.insert(0, Line2D([0], [0], color='k', lw=1.5, ls='--', alpha=0.5, label='Referência (GT)'))

ax.legend(handles=legend_elements, loc='upper right', framealpha=0.95, fontsize=10, shadow=True)
plt.title(f"SLAM Final: Visualização Limpa\n(Triângulos = Poses da Câmera)")
plt.xlabel("X (m)"); plt.ylabel("Y (m)")
plt.axis('equal'); plt.grid(True)

plt.savefig("slam_small_triangles.png", dpi=150, bbox_inches='tight')
print("Salvo: slam_small_triangles.png")
plt.show()
