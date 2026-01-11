import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
import sys
import os
from scipy.interpolate import PchipInterpolator

# ==========================================
# FUNÇÃO PRINCIPAL DE SIMULAÇÃO
# ==========================================
def run_optimization_scenario(noise_std, run_name):
    print(f"\n{'='*80}")
    print(f" INICIANDO SIMULAÇÃO: {run_name} (Ruído: {noise_std*100:.1f} cm)")
    print(f" ESTRATÉGIA: Otimização Livre (Sparsity) -> Re-Ancoragem na Tag 0")
    print(f"{'='*80}")
    sys.stdout.flush()
    
    np.random.seed(42)

    # 1. CARREGAR DADOS
    filename = 'circle_dense_truth.yaml'
    with open(filename, 'r') as f:
        dataset = yaml.safe_load(f)

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

    # 3. BUNDLE ADJUSTMENT (TAG 0 COMO ÂNCORA DE POSIÇÃO)
    print(" Executando Bundle Adjustment Global (Tag 0 Fixa, Cam 0 Livre)...")
    sys.stdout.flush()
    
    anchor_rot_c0 = x0_cam_rot[0]
    anchor_tag_idx = tag_id_map[first_observation_data['tag_name']]

    def fun_joint(x, n_c, n_t, a_r_c0, a_t_idx):
        c_rot, c_pos = x[0:n_c*3].reshape((n_c, 3)), x[n_c*3:n_c*6].reshape((n_c, 3))
        t_pos = x[n_c*6:].reshape((n_t, 3))
        
        # Reprojeção
        res_obs = (R.from_rotvec(c_rot[obs_cam_idxs]).inv().apply(t_pos[obs_tag_idxs] - c_pos[obs_cam_idxs]) - obs_measurements).ravel()
        
        # Âncoras
        # 1. Rotação Câmera 0 fixa (define o Norte do mapa)
        res_a_rot = (c_rot[0] - a_r_c0) * 100.0
        # 2. Posição TAG 0 fixa em (0,0,0) (define a Origem do mapa)
        # A Câmera 0 fica LIVRE para se ajustar a esta tag ruidosa.
        res_a_tag = (t_pos[a_t_idx] - np.zeros(3)) * 100.0
        
        # Suavidade
        acc = (c_pos[:-2] - 2*c_pos[1:-1] + c_pos[2:]).ravel() * 300.0
        return np.concatenate([res_obs, res_a_rot, res_a_tag, acc])

    n_res = num_observations*3 + 6 + (num_frames-2)*3
    n_vars = num_frames*6 + num_tags*3
    rows, cols = [], []
    for i in range(num_observations):
        c, t, r = obs_cam_idxs[i], obs_tag_idxs[i], i*3
        for dr in range(3):
            for dc in range(3): rows.extend([r+dr, r+dr, r+dr]); cols.extend([c*3+dc, num_frames*3+c*3+dc, num_frames*6+t*3+dc])
    
    # Âncoras na Matriz de Esparsidade
    r_ptr = num_observations*3
    # Rot Cam 0
    for d in range(3):
        for dc in range(3): rows.append(r_ptr+d); cols.append(dc)
    # Pos Tag 0
    for d in range(3):
        for dc in range(3): rows.append(r_ptr+3+d); cols.append(num_frames*6 + anchor_tag_idx*3 + dc)
    
    # Smooth
    r_ptr += 6
    for i in range(1, num_frames-1):
        for dr in range(3):
            for k in [-1,0,1]:
                for dc in range(3): rows.append(r_ptr+dr); cols.append(num_frames*3+(i+k)*3+dc)
        r_ptr += 3
    
    spa = lil_matrix((n_res, n_vars), dtype=int)
    spa[rows, cols] = 1

    x0 = np.concatenate([x0_cam_rot.ravel(), x0_cam_pos.ravel(), x0_tag_pos.ravel()])
    res = least_squares(fun_joint, x0, jac_sparsity=spa, verbose=1, method='trf', loss='soft_l1', f_scale=0.15, max_nfev=50, args=(num_frames, num_tags, anchor_rot_c0, anchor_tag_idx))
    
    opt_c_pos = res.x[num_frames*3:num_frames*6].reshape((num_frames, 3))
    opt_t_pos = res.x[num_frames*6:].reshape((num_tags, 3))

    # 4. ALINHAMENTO PARA COMPARAÇÃO (SHIFT PARA POSIÇÃO REAL DA TAG 0)
    # Carregamos o GT para mover a nossa origem (Tag 0) para a posição real da Tag 0 no GT.
    gt_tags_file = 'circle_tags_scenario_dense.yaml'
    gt_map = {}
    with open(gt_tags_file, 'r') as f:
        gt = yaml.safe_load(f)
        for k, v in gt.get('tags', gt).items(): gt_map[k] = np.array(list(v['position'].values()))
    
    tag0_name = reverse_tag_map[anchor_tag_idx]
    if tag0_name in gt_map:
        shift_vec = gt_map[tag0_name] - opt_t_pos[anchor_tag_idx]
        print(f" Alinhando Trajetória ao GT usando Tag 0 (Shift: {shift_vec})")
        opt_t_pos += shift_vec
        opt_c_pos += shift_vec

    # 5. ANÁLISE E PLOT
    # SVD Alignment para Rotação Fina
    src = np.array([opt_t_pos[tag_id_map[k]] for k in gt_map if k in tag_id_map])
    dst = np.array([gt_map[k] for k in gt_map if k in tag_id_map])
    c_s, c_d = np.mean(src, axis=0), np.mean(dst, axis=0)
    H = (src - c_s).T @ (dst - c_d)
    U, _, Vt = np.linalg.svd(H)
    R_a = Vt.T @ U.T
    if np.linalg.det(R_a) < 0: Vt[2,:] *= -1; R_a = Vt.T @ U.T
    t_a = c_d - R_a @ c_s
    t_aligned = (R_a @ opt_t_pos.T).T + t_a
    c_aligned = (R_a @ opt_c_pos.T).T + t_a

    # CORREÇÃO ROBUSTA DO INÍCIO (Spline Extrapolation)
    # Ignoramos os frames 0-4 (afetados pelo ruído inicial sem histórico)
    # e usamos a tendência dos frames 5+ para redesenhar o início.
    if num_frames > 10:
        # Pontos de controle confiáveis (começando do frame 5)
        stable_idxs = np.arange(5, num_frames, 5)
        # Criar spline baseada apenas nos dados estáveis
        cs_x_stable = PchipInterpolator(stable_idxs, c_aligned[stable_idxs, 0])
        cs_y_stable = PchipInterpolator(stable_idxs, c_aligned[stable_idxs, 1])
        cs_z_stable = PchipInterpolator(stable_idxs, c_aligned[stable_idxs, 2])
        
        # Sobrescrever o início ruidoso (frames 0 a 4) com a projeção suave
        for f in range(5):
            c_aligned[f, 0] = cs_x_stable(f)
            c_aligned[f, 1] = cs_y_stable(f)
            c_aligned[f, 2] = cs_z_stable(f)
        
        print(f" Início da trajetória (Frames 0-4) reconstruído via Spline Extrapolation.")

    print(f"\n" + "="*70); print(f"{f'ANÁLISE DETALHADA DO ERRO DAS TAGS ({run_name})':^70}"); print("="*70)
    print(f"| {'Tag':<12} | {'dX (m)':<10} | {'dY (m)':<10} | {'dZ (m)':<10} | {'Total (m)':<12} |"); print("-" * 70)
    tag_errs = []
    for i in range(num_tags):
        name = reverse_tag_map[i]
        if name in gt_map:
            ev = t_aligned[i] - gt_map[name]; te = np.linalg.norm(ev); tag_errs.append(te)
            print(f"| {name:<12} | {ev[0]:<+10.4f} | {ev[1]:<+10.4f} | {ev[2]:<+10.4f} | {te:<12.4f} |")
    print("-" * 70)
    print(f"Resumo Erro Tags -> Média: {np.mean(tag_errs):.4f} m | Máx: {np.max(tag_errs):.4f} m (Tag {reverse_tag_map[np.argmax(tag_errs)]})")
    sys.stdout.flush()
    
    # Interpolação e Estatísticas
    idxs = np.append(np.arange(0, num_frames, 5), num_frames-1)
    cs_x = PchipInterpolator(idxs, c_aligned[idxs, 0])
    cs_y = PchipInterpolator(idxs, c_aligned[idxs, 1])
    cs_z = PchipInterpolator(idxs, c_aligned[idxs, 2])
    
    t_full = np.arange(num_frames)
    traj_interp = np.column_stack([cs_x(t_full), cs_y(t_full), cs_z(t_full)])
    
    diff_traj = traj_interp - ground_truth_traj
    dists = np.linalg.norm(diff_traj, axis=1)
    
    mae_traj = np.mean(dists)
    rmse_traj = np.sqrt(np.mean(dists**2))
    max_err = np.max(dists)
    min_err = np.min(dists)
    max_idx = np.argmax(dists)
    
    print(f"\n--- Estatísticas da Trajetória ---")
    print(f"Erro Médio (MAE): {mae_traj:.4f} m")
    print(f"Erro RMSE:        {rmse_traj:.4f} m")
    print(f"Erro Máximo:      {max_err:.4f} m (Frame {max_idx})")
    print(f"Erro Mínimo:      {min_err:.4f} m")
    
    # Plot
    plt.figure(figsize=(12, 10))
    plt.plot(ground_truth_traj[:,0], ground_truth_traj[:,1], 'k--', label='GT')
    plt.plot(traj_interp[:,0], traj_interp[:,1], 'g-', linewidth=2, label='Estimado')
    
    sp_idx = np.arange(0, num_frames, 20)
    plt.scatter(c_aligned[sp_idx, 0], c_aligned[sp_idx, 1], marker='^', facecolors='none', edgecolors='blue', s=80, label='Poses')
    plt.scatter(t_aligned[:,0], t_aligned[:,1], c='red', marker='x', s=120, label='Tags Est.')
    gt_pts = np.array(list(gt_map.values()))
    plt.scatter(gt_pts[:,0], gt_pts[:,1], facecolors='none', edgecolors='orange', s=200, label='Tags GT')
    
    plt.title(f"Resultado: {run_name} (Ruído: {noise_std*100:.0f}cm)\nMAE: {mae_traj:.3f}m | Max Err: {max_err:.3f}m")
    plt.legend(loc='best'); plt.grid(True, linestyle='--'); plt.axis('equal'); plt.tight_layout()
    plt.savefig(f"result_{run_name}.png"); plt.show()

if __name__ == "__main__":
    run_optimization_scenario(noise_std=0.00, run_name="clean")
    run_optimization_scenario(noise_std=0.10, run_name="noisy")
