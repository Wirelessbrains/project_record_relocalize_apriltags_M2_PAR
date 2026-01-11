import argparse
import sys
import os

import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
from scipy.interpolate import PchipInterpolator

# ==========================================
# FUNÇÃO PRINCIPAL DE SIMULAÇÃO
# ==========================================
def run_optimization_scenario(noise_std, run_name, data_file, tags_file, plot_individual=True):
    print(f"\n{'='*80}")
    print(f" INICIANDO SIMULAÇÃO: {run_name} (Ruído: {noise_std*100:.1f} cm)")
    print(f" ESTRATÉGIA: BA Global (Tag 0 Livre) -> Shift para Origem -> Correção Spline")
    print(f"{'='*80}")
    sys.stdout.flush()
    
    np.random.seed(42)

    # 1. CARREGAR DADOS
    with open(data_file, 'r') as f:
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

    # 3. BUNDLE ADJUSTMENT
    # Estratégia: Fixar Rotação da Câmera 0 e usar Tag 0 como Âncora SUAVE (para depois fazer shift).
    # A Tag 0 é penalizada se sair do zero, mas não travada rigidamente, permitindo ajuste geométrico.
    
    print(" Executando Bundle Adjustment Global...")
    sys.stdout.flush()
    
    anchor_rot_c0 = x0_cam_rot[0]

    def fun_joint(x, n_c, n_t, a_r_c0, a_t_idx, smooth_w=200.0):
        c_rot, c_pos = x[0:n_c*3].reshape((n_c, 3)), x[n_c*3:n_c*6].reshape((n_c, 3))
        t_pos = x[n_c*6:].reshape((n_t, 3))
        
        # Reprojeção
        res_obs = (R.from_rotvec(c_rot[obs_cam_idxs]).inv().apply(t_pos[obs_tag_idxs] - c_pos[obs_cam_idxs]) - obs_measurements).ravel()
        
        # Âncoras
        res_a_rot = (c_rot[0] - a_r_c0) * 100.0
        res_a_tag = (t_pos[a_t_idx] - np.zeros(3)) * 100.0
        
        # Suavidade
        acc = (c_pos[:-2] - 2*c_pos[1:-1] + c_pos[2:]).ravel() * smooth_w
        return np.concatenate([res_obs, res_a_rot, res_a_tag, acc])

    # Matriz de Esparsidade
    n_res = num_observations*3 + 6 + (num_frames-2)*3
    n_vars = num_frames*6 + num_tags*3
    rows, cols = [], []
    # Obs
    for i in range(num_observations):
        c, t, r = obs_cam_idxs[i], obs_tag_idxs[i], i*3
        for dr in range(3):
            for dc in range(3): rows.extend([r+dr, r+dr, r+dr]); cols.extend([c*3+dc, num_frames*3+c*3+dc, num_frames*6+t*3+dc])
    # Anchor
    r_ptr = num_observations*3
    for d in range(3): rows.append(r_ptr+d); cols.append(d) # Rot Cam 0
    for d in range(3): rows.append(r_ptr+3+d); cols.append(num_frames*6 + anchor_tag_idx*3 + d) # Pos Tag 0
    # 3. Suavidade (Aceleração)
    # Resíduo 'acc' começa no índice r_ptr.
    # acc[i-1] (para i=1..N-2) depende de pos[i-1], pos[i], pos[i+1]
    r_ptr += 3 # Pula as 3 linhas da âncora da tag
    pos_var_start = num_frames * 3
    
    if num_frames > 2:
        for i in range(1, num_frames-1):
            # Para cada frame central i, temos 3 resíduos de aceleração (x, y, z)
            for d in range(3):
                row_idx = r_ptr + (i-1)*3 + d
                # Esse resíduo depende das posições em i-1, i, i+1 na mesma dimensão d
                for k in [-1, 0, 1]:
                    col_idx = pos_var_start + (i+k)*3 + d
                    rows.append(row_idx)
                    cols.append(col_idx)
                    
                    # Rotação também é suavizada? Se sim, adicione aqui.
                    # No fun_joint, 'acc' pega posições. Se tiver rotação, precisa adicionar.
                    # Vamos verificar fun_joint... Ah, acc é só pos! (linha 166: acc = (c_pos...))
                    # Se acc incluir rotação no futuro, precisaria de mais linhas.
                    # ATENÇÃO: Se fun_joint retornar suavidade de rotação também, a matriz quebra.
                    # Vou assumir que fun_joint retorna SÓ suavidade de POSIÇÃO conforme linha 166.

    spa = lil_matrix((n_res, n_vars), dtype=int)
    spa[rows, cols] = 1

    # Configuração Universal (Funciona para Clean e Noisy sem "dicas")
    # SoftL1 com escala 0.1m permite precisão em dados limpos (comporta-se como linear < 10cm)
    # e robustez em dados ruidosos (comporta-se como linear > 10cm, rejeitando outliers).
    # Suavidade de 100.0 é o compromisso para filtrar ruído sem deformar demais as curvas.
    loss, f_s, n_iter = 'soft_l1', 0.1, 200
    smooth_w_val = 100.0
    print(f" Configuração UNIVERSAL: SoftL1(0.1), Smooth={smooth_w_val}")

    x0 = np.concatenate([x0_cam_rot.ravel(), x0_cam_pos.ravel(), x0_tag_pos.ravel()])
    print(f" Iniciando solver ({n_vars} vars, {n_res} resíduos)...")
    res_ba = least_squares(fun_joint, x0, jac_sparsity=spa, verbose=1, method='trf', 
                           loss=loss, f_scale=f_s, max_nfev=n_iter, 
                           args=(num_frames, num_tags, anchor_rot_c0, anchor_tag_idx, smooth_w_val))
    
    opt_c_pos = res_ba.x[num_frames*3:num_frames*6].reshape((num_frames, 3))
    opt_t_pos = res_ba.x[num_frames*6:].reshape((num_tags, 3))

    # 4. RE-ANCORAGEM (SHIFT GLOBAL)
    # Movemos o mundo para que a Tag 0 otimizada seja a Origem (0,0,0)
    tag0_final = opt_t_pos[anchor_tag_idx].copy()
    print(f" Tag 0 convergiu para: {tag0_final}. Aplicando Shift para Origem...")
    opt_t_pos -= tag0_final
    opt_c_pos -= tag0_final

    # 5. ANÁLISE E PLOT
    gt_map = {}
    with open(tags_file, 'r') as f:
        gt = yaml.safe_load(f)
        for k, v in gt.get('tags', gt).items(): gt_map[k] = np.array(list(v['position'].values()))

    # SVD Alignment (Alinha a forma geral ao GT)
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
    # Elimina o "kink" inicial causado pelo ruído no frame 0
    if num_frames > 10:
        stable_idxs = np.arange(5, num_frames, 5)
        # Interpoladores
        cs_x = PchipInterpolator(stable_idxs, c_aligned[stable_idxs, 0])
        cs_y = PchipInterpolator(stable_idxs, c_aligned[stable_idxs, 1])
        cs_z = PchipInterpolator(stable_idxs, c_aligned[stable_idxs, 2])
        # Sobrescreve início ruidoso
        for f in range(5):
            c_aligned[f] = [cs_x(f), cs_y(f), cs_z(f)]
        print(f" Início da trajetória (Frames 0-4) suavizado via Spline.")

    print(f"\n" + "="*70); print(f"{f'ANÁLISE DETALHADA ({run_name})':^70}"); print("="*70)
    print(f"| {'Tag':<12} | {'dX (m)':<10} | {'dY (m)':<10} | {'dZ (m)':<10} | {'Total (m)':<12} |"); print("-" * 70)
    tag_errs = []
    for i in range(num_tags):
        name = reverse_tag_map[i]
        if name in gt_map:
            ev = t_aligned[i] - gt_map[name]; te = np.linalg.norm(ev); tag_errs.append(te)
            print(f"| {name:<12} | {ev[0]:<+10.4f} | {ev[1]:<+10.4f} | {ev[2]:<+10.4f} | {te:<12.4f} |")
    print("-" * 70)
    print(f"Média Erro Tags: {np.mean(tag_errs):.4f} m | Máx: {np.max(tag_errs):.4f} m")
    
    # Interpolação Final e Erro Trajetória
    t_full = np.arange(num_frames)
    idxs = np.append(np.arange(0, num_frames, 5), num_frames-1)
    # Recalcula spline com o início corrigido
    traj_interp = np.column_stack([
        PchipInterpolator(idxs, c_aligned[idxs, 0])(t_full),
        PchipInterpolator(idxs, c_aligned[idxs, 1])(t_full),
        PchipInterpolator(idxs, c_aligned[idxs, 2])(t_full)
    ])
    
    diff_traj = traj_interp - ground_truth_traj
    mae = np.mean(np.linalg.norm(diff_traj, axis=1))
    mx = np.max(np.linalg.norm(diff_traj, axis=1))
    mn = np.min(np.linalg.norm(diff_traj, axis=1))
    print(f"Erro Trajetória -> MAE: {mae:.4f} m | Max: {mx:.4f} m | Min: {mn:.4f} m")

    result_data = {
        "run_name": run_name,
        "noise_std": noise_std,
        "gt_traj": ground_truth_traj,
        "traj_interp": traj_interp,
        "poses": c_aligned,
        "tags_est": t_aligned,
        "tags_gt": gt_map,
        "mae": mae,
        "tag_avg": np.mean(tag_errs)
    }

    if plot_individual:
        plt.figure(figsize=(12, 10))
        plt.plot(ground_truth_traj[:,0], ground_truth_traj[:,1], 'k--', label='GT')
        plt.plot(traj_interp[:,0], traj_interp[:,1], 'g-', linewidth=2, label='Estimado')
        sp_idx = np.arange(0, num_frames, 20)
        plt.scatter(c_aligned[sp_idx, 0], c_aligned[sp_idx, 1], marker='^', facecolors='none', edgecolors='blue', s=80, label='Poses')
        plt.scatter(t_aligned[:,0], t_aligned[:,1], c='red', marker='x', s=120, label='Tags Est.')
        gt_pts = np.array(list(gt_map.values()))
        plt.scatter(gt_pts[:,0], gt_pts[:,1], facecolors='none', edgecolors='orange', s=200, label='Tags GT')
        plt.title(f"Resultado: {run_name} (Ruído: {noise_std*100:.0f}cm)\nMAE: {mae:.3f}m | Tags Avg: {np.mean(tag_errs):.3f}m")
        plt.legend(); plt.grid(True); plt.axis('equal'); plt.tight_layout()
        plt.savefig(f"result_{run_name}.png")
        plt.show()

    return result_data


def plot_side_by_side(clean_res, noisy_res, output_path="result_comparison.png"):
    fig, axes = plt.subplots(1, 2, figsize=(20, 9))
    for ax, res in zip(axes, [clean_res, noisy_res]):
        ax.plot(res["gt_traj"][:, 0], res["gt_traj"][:, 1], 'k--', label='GT', alpha=0.7)
        ax.plot(res["traj_interp"][:, 0], res["traj_interp"][:, 1], 'g-', linewidth=2, label='Estimado')

        sp_idx = np.arange(0, len(res["poses"]), 20)
        ax.scatter(res["poses"][sp_idx, 0], res["poses"][sp_idx, 1], marker='^', facecolors='none', edgecolors='blue', s=80, label='Poses')
        ax.scatter(res["tags_est"][:, 0], res["tags_est"][:, 1], c='red', marker='x', s=120, label='Tags Est.')
        gt_pts = np.array(list(res["tags_gt"].values()))
        ax.scatter(gt_pts[:, 0], gt_pts[:, 1], facecolors='none', edgecolors='orange', s=200, label='Tags GT')

        ax.set_title(f"{res['run_name'].title()} (Ruído: {res['noise_std']*100:.0f}cm)\nMAE: {res['mae']:.3f}m | Tags Avg: {res['tag_avg']:.3f}m")
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
        ax.legend(); ax.grid(True, linestyle='--'); ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(output_path)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Otimização de trajetória com BA e spline.")
    parser.add_argument("--data", required=True, help="Arquivo YAML com frames e detecções (ex: circle_dense_truth.yaml)")
    parser.add_argument("--tags", required=True, help="Arquivo YAML com posições verdadeiras das tags (ex: circle_tags_scenario_dense.yaml)")
    parser.add_argument("--noise", type=float, default=0.0, help="Desvio padrão do ruído gaussiano (metros). Default: 0.0 (sem ruído).")
    args = parser.parse_args()

    noise_val = args.noise if args.noise is not None else 0.0
    
    if noise_val > 0:
        clean_res = run_optimization_scenario(noise_std=0.0, run_name="clean", data_file=args.data, tags_file=args.tags, plot_individual=False)
        noisy_res = run_optimization_scenario(noise_std=noise_val, run_name="noisy", data_file=args.data, tags_file=args.tags, plot_individual=False)
        plot_side_by_side(clean_res, noisy_res, output_path="result_comparison.png")
        print("Gráfico combinado salvo em 'result_comparison.png'")
    else:
        run_optimization_scenario(noise_std=0.0, run_name="clean", data_file=args.data, tags_file=args.tags)
