import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
import sys
import os
import argparse
from scipy.interpolate import PchipInterpolator

# ==========================================
# CLASSE DE RESULTADO
# ==========================================
class SimResult:
    def __init__(self, run_name, noise_std, gt_traj, est_traj, est_tags, gt_map, mae, tag_avg, c_aligned):
        self.name = run_name
        self.noise = noise_std
        self.gt_traj = gt_traj
        self.est_traj = est_traj
        self.est_tags = est_tags
        self.gt_map = gt_map
        self.mae = mae
        self.tag_avg = tag_avg
        self.c_aligned = c_aligned # Poses esparsas alinhadas

# ==========================================
# FUNÇÃO PRINCIPAL DE SIMULAÇÃO (CORE)
# ==========================================
def run_optimization_scenario(dataset_data, gt_data, noise_std, run_name):
    print(f"\n{'='*80}")
    print(f" INICIANDO SIMULAÇÃO: {run_name} (Ruído: {noise_std*100:.1f} cm)")
    print(f" ESTRATÉGIA: BA Global (Tag 0 Livre) -> Shift para Origem -> Correção Spline")
    print(f"{'='*80}")
    sys.stdout.flush()
    
    np.random.seed(42)

    # 1. SETUP DE DADOS
    frames = dataset_data['frames']
    gt_map = gt_data

    tag_id_map, next_tag_idx, reverse_tag_map = {}, 0, {}
    obs_cam_idxs, obs_tag_idxs, obs_measurements, ground_truth_traj = [], [], [], []
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
                noise = np.random.normal(0, noise_std, 3)
                first_observation_data = {'cam_gt_rot': R_c_input, 'meas_local': first_meas + noise, 'tag_name': first_tag_name}

            for tag_name, pose_data in detection_field.items():
                if tag_name not in tag_id_map:
                    tag_id_map[tag_name] = next_tag_idx
                    reverse_tag_map[next_tag_idx] = tag_name
                    next_tag_idx += 1
                obs_cam_idxs.append(i)
                obs_tag_idxs.append(tag_id_map[tag_name])
                raw_meas = np.array([pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z']])
                noise = np.random.normal(0, noise_std, 3)
                obs_measurements.append(raw_meas + noise)

    obs_cam_idxs = np.array(obs_cam_idxs, dtype=np.int32)
    obs_tag_idxs = np.array(obs_tag_idxs, dtype=np.int32)
    obs_measurements = np.array(obs_measurements, dtype=np.float64)
    ground_truth_traj = np.array(ground_truth_traj)
    num_frames = len(frames)
    num_tags, num_observations = len(tag_id_map), len(obs_cam_idxs)
    anchor_tag_idx = tag_id_map[first_observation_data['tag_name']]

    # 2. INICIALIZAÇÃO
    P_tag_anchor_world = np.zeros(3)
    R_cam_anchor = first_observation_data['cam_gt_rot']
    P_tag_local = first_observation_data['meas_local']
    start_pos_vec = P_tag_anchor_world - R_cam_anchor.apply(P_tag_local)
    
    x0_cam_pos, x0_cam_rot = np.zeros((num_frames, 3)), np.zeros((num_frames, 3))
    for i, frame in enumerate(frames):
        p, q = frame['pose']['position'], frame['pose']['orientation']
        x0_cam_pos[i], x0_cam_rot[i] = [p['x'], p['y'], p['z']], R.from_quat([q['x'], q['y'], q['z'], q['w']]).as_rotvec()
    
    idx_start = obs_cam_idxs[0]
    R_off = R.from_rotvec(R_cam_anchor.as_rotvec()) * R.from_rotvec(x0_cam_rot[idx_start]).inv()
    for i in range(num_frames):
        x0_cam_pos[i] = R_off.apply(x0_cam_pos[i] - x0_cam_pos[idx_start]) + start_pos_vec
        x0_cam_rot[i] = (R_off * R.from_rotvec(x0_cam_rot[i])).as_rotvec()

    tag_projs = [[] for _ in range(num_tags)]
    for i in range(num_observations):
        R_c, t_c = R.from_rotvec(x0_cam_rot[obs_cam_idxs[i]]), x0_cam_pos[obs_cam_idxs[i]]
        tag_projs[obs_tag_idxs[i]].append(t_c + R_c.apply(obs_measurements[i]))
    x0_tag_pos = np.zeros((num_tags, 3))
    for t in range(num_tags):
        if tag_projs[t]: x0_tag_pos[t] = np.median(tag_projs[t], axis=0)

    # 3. BUNDLE ADJUSTMENT
    print(" Executando Bundle Adjustment Global...")
    anchor_rot_c0 = x0_cam_rot[0]

    def fun_joint(x, n_c, n_t, a_r_c0, a_t_idx, smooth_w=100.0):
        c_rot, c_pos = x[0:n_c*3].reshape((n_c, 3)), x[n_c*3:n_c*6].reshape((n_c, 3))
        t_pos = x[n_c*6:].reshape((n_t, 3))
        res_obs = (R.from_rotvec(c_rot[obs_cam_idxs]).inv().apply(t_pos[obs_tag_idxs] - c_pos[obs_cam_idxs]) - obs_measurements).ravel()
        res_a_rot = (c_rot[0] - a_r_c0) * 100.0
        res_a_tag = (t_pos[a_t_idx] - np.zeros(3)) * 1.0 # Âncora fraca
        acc = (c_pos[:-2] - 2*c_pos[1:-1] + c_pos[2:]).ravel() * smooth_w
        return np.concatenate([res_obs, res_a_rot, res_a_tag, acc])

    n_res = num_observations*3 + 6 + (num_frames-2)*3
    n_vars = num_frames*6 + num_tags*3
    rows, cols = [], []
    for i in range(num_observations):
        c, t, r = obs_cam_idxs[i], obs_tag_idxs[i], i*3
        for dr in range(3):
            for dc in range(3): rows.extend([r+dr]*2); cols.extend([c*3+dc, num_frames*3+c*3+dc]); rows.append(r+dr); cols.append(num_frames*6+t*3+dc)
    r_ptr = num_observations*3
    for d in range(3): rows.append(r_ptr+d); cols.append(d)
    for d in range(3): rows.append(r_ptr+3+d); cols.append(num_frames*6 + anchor_tag_idx*3 + d)
    r_ptr += 6
    if num_frames > 2:
        for i in range(1, num_frames-1):
            for d in range(3):
                for k in [-1,0,1]: rows.append(r_ptr+d); cols.append(num_frames*3+(i+k)*3+d)
            r_ptr += 3
    
    spa = lil_matrix((n_res, n_vars), dtype=int); spa[rows, cols] = 1

    # Configuração Universal
    loss, f_s, n_iter, smooth_w_val = 'soft_l1', 0.1, 200, 100.0
    
    x0 = np.concatenate([x0_cam_rot.ravel(), x0_cam_pos.ravel(), x0_tag_pos.ravel()])
    print(f" Iniciando solver ({n_vars} vars, {n_res} resíduos)...")
    res_ba = least_squares(fun_joint, x0, jac_sparsity=spa, verbose=1, method='trf', 
                           loss=loss, f_scale=f_s, max_nfev=n_iter, 
                           args=(num_frames, num_tags, anchor_rot_c0, anchor_tag_idx, smooth_w_val))
    
    opt_c_pos = res_ba.x[num_frames*3:num_frames*6].reshape((num_frames, 3))
    opt_t_pos = res_ba.x[num_frames*6:].reshape((num_tags, 3))

    # 4. SHIFT GLOBAL
    tag0_final = opt_t_pos[anchor_tag_idx].copy()
    print(f" Tag 0 convergiu para: {tag0_final}. Aplicando Shift para Origem...")
    opt_t_pos -= tag0_final
    opt_c_pos -= tag0_final

    # 5. ALINHAMENTO
    src = np.array([opt_t_pos[tag_id_map[k]] for k in gt_map if k in tag_id_map])
    dst = np.array([gt_map[k] for k in gt_map if k in tag_id_map])
    cs, cd = np.mean(src, axis=0), np.mean(dst, axis=0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    R_a = Vt.T @ U.T
    if np.linalg.det(R_a) < 0: Vt[2,:] *= -1; R_a = Vt.T @ U.T
    t_a = cd - R_a @ cs
    t_aligned = (R_a @ opt_t_pos.T).T + t_a
    c_aligned = (R_a @ opt_c_pos.T).T + t_a

    # Correção Spline Início
    if num_frames > 10:
        si = np.arange(5, num_frames, 5)
        for d in range(3):
            spline = PchipInterpolator(si, c_aligned[si, d])
            c_aligned[0:5, d] = spline(np.arange(5))

    # Relatório Detalhado
    print(f"\n{f'ANÁLISE DETALHADA ({run_name})':^70}")
    print(f"| {'Tag':<12} | {'dX (m)':<10} | {'dY (m)':<10} | {'dZ (m)':<10} | {'Total (m)':<12} |")
    print("-" * 70)
    tag_errs = []
    for i in range(num_tags):
        nm = reverse_tag_map[i]
        if nm in gt_map:
            ev = t_aligned[i] - gt_map[nm]; te = np.linalg.norm(ev); tag_errs.append(te)
            print(f"| {nm:<12} | {ev[0]:<+10.4f} | {ev[1]:<+10.4f} | {ev[2]:<+10.4f} | {te:<12.4f} |")
    print("-" * 70)
    
    t_full = np.arange(num_frames)
    idxs = np.append(np.arange(0, num_frames, 5), num_frames-1)
    traj_interp = np.column_stack([PchipInterpolator(idxs, c_aligned[idxs, d])(t_full) for d in range(3)])
    
    diff_traj = traj_interp - ground_truth_traj
    mae = np.mean(np.linalg.norm(diff_traj, axis=1))
    mx = np.max(np.linalg.norm(diff_traj, axis=1))
    mn = np.min(np.linalg.norm(diff_traj, axis=1))
    print(f"Trajetória -> MAE: {mae:.4f}m | Max: {mx:.4f}m | Min: {mn:.4f}m")

    return SimResult(run_name, noise_std, ground_truth_traj, traj_interp, t_aligned, gt_map, mae, np.mean(tag_errs), c_aligned)

# ==========================================
# EXECUÇÃO PRINCIPAL (CLI)
# ==========================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", default="circle_dense_truth.yaml")
    parser.add_argument("--scenario", default="circle_tags_scenario_dense.yaml")
    parser.add_argument("--noise", type=float, default=0.1)
    args = parser.parse_args()

    with open(args.trajectory, 'r') as f: ds_data = yaml.safe_load(f)
    with open(args.scenario, 'r') as f: gt_scen = yaml.safe_load(f)
    gt_map = {k: np.array(list(v['position'].values())) for k, v in gt_scen.get('tags', gt_scen).items()}

    res_clean = run_optimization_scenario(ds_data, gt_map, 0.0, "Clean")
    res_noisy = run_optimization_scenario(ds_data, gt_map, args.noise, "Noisy")

    fig, axes = plt.subplots(1, 2, figsize=(20, 9))
    for ax, res in zip(axes, [res_clean, res_noisy]):
        ax.plot(res.gt_traj[:,0], res.gt_traj[:,1], 'k--', label='GT', alpha=0.7)
        ax.plot(res.est_traj[:,0], res.est_traj[:,1], 'g-', linewidth=2, label='Estimado')
        
        # Poses
        sp_idx = np.arange(0, len(res.c_aligned), 20)
        ax.scatter(res.c_aligned[sp_idx,0], res.c_aligned[sp_idx,1], marker='^', facecolors='none', edgecolors='blue', s=60, label='Poses')
        
        ax.scatter(res.est_tags[:,0], res.est_tags[:,1], c='red', marker='x', s=100, label='Tags Est.')
        gt_pts = np.array(list(res.gt_map.values()))
        ax.scatter(gt_pts[:,0], gt_pts[:,1], facecolors='none', edgecolors='orange', s=150, label='Tags GT')
        
        ax.set_title(f"{res.name} (Ruído {res.noise*100:.0f}cm)\nMAE: {res.mae:.3f}m | Tags Avg: {res.tag_avg:.3f}m")
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
        ax.legend(); ax.grid(True, linestyle='--'); ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig("comparison_clean_vs_noisy.png")
    print("\nGráfico salvo em: comparison_clean_vs_noisy.png")
    plt.show()
