import numpy as np
import matplotlib.pyplot as plt
import re
import sys

# =============================================================================
# 1. PARSER E LEITURA (Mantido igual)
# =============================================================================
def parse_yaml_manual(filename):
    try:
        with open(filename, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"ERRO: Arquivo '{filename}' não encontrado.")
        sys.exit(1)

    poses_db = {}
    tag_pattern = re.compile(
        r'name:\s*(tag36h11:\d+).*?'
        r'position:\s*.*?'
        r'x:\s*([\d\.e\-\+]+).*?'
        r'y:\s*([\d\.e\-\+]+).*?'
        r'z:\s*([\d\.e\-\+]+)', 
        re.DOTALL
    )
    matches = tag_pattern.findall(content)
    for name, x, y, z in matches:
        poses_db[name] = np.array([float(x), float(y), float(z)])
    return poses_db

def load_ground_truth(filename):
    poses_db = parse_yaml_manual(filename)
    gt_camera_pos = np.array([0.0, 0.0, 0.0])
    
    clean_poses_list = []
    sorted_names = sorted(poses_db.keys(), key=lambda x: int(x.split(':')[-1]))
    for name in sorted_names:
        clean_poses_list.append(poses_db[name])

    clean_poses = np.array(clean_poses_list)
    real_distances = np.linalg.norm(clean_poses - gt_camera_pos, axis=1)
    return clean_poses, real_distances, gt_camera_pos

# =============================================================================
# 2. OTIMIZAÇÃO CORRIGIDA (Com verificação rigorosa de parada)
# =============================================================================
def gradient_descent(map_poses, measured_dists, initial_guess, lr=0.01, max_iter=5000, stop_threshold=1e-5):
    """
    Agora retorna explicitamente o MOTIVO da parada.
    """
    est_pos = initial_guess.copy()
    history = [est_pos.copy()]
    
    reason = "Max Iterations" # Assumimos o pior caso
    final_grad_norm = 0.0
    
    for i in range(max_iter):
        diff = est_pos - map_poses 
        calc_dists = np.linalg.norm(diff, axis=1)
        safe_dists = calc_dists.copy()
        safe_dists[safe_dists < 1e-9] = 1e-9
        
        error = calc_dists - measured_dists
        
        # Gradiente
        grad = 2 * np.sum(error[:, np.newaxis] * (diff / safe_dists[:, np.newaxis]), axis=0)
        final_grad_norm = np.linalg.norm(grad)
        
        # Passo de atualização
        step = lr * grad
        est_pos = est_pos - step
        history.append(est_pos.copy())
        
        # --- CRITÉRIOS DE PARADA RIGOROSOS ---
        
        # 1. O gradiente é pequeno o suficiente? (Matematicamente convergiu)
        if final_grad_norm < stop_threshold:
            reason = "Tolerância Atingida (Gradiente)"
            return est_pos, np.array(history), i+1, reason, final_grad_norm
            
        # 2. O passo foi insignificante? (Não está mais saindo do lugar)
        # Isso ajuda quando o gradiente oscila mas o robô não anda
        if np.linalg.norm(step) < stop_threshold:
            reason = "Tolerância Atingida (Passo Mínimo)"
            return est_pos, np.array(history), i+1, reason, final_grad_norm

    return est_pos, np.array(history), max_iter, reason, final_grad_norm

# =============================================================================
# 3. PLOTAGEM DETALHADA
# =============================================================================
def plot_simulation(map_poses, gt_poses, gt_cam, est_cam, history, iterations, reason, title, filename):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Tags
    ax.scatter(map_poses[:,0], map_poses[:,1], map_poses[:,2], c='black', marker='s', s=40, label='Tags (Mapa)')
    if not np.array_equal(map_poses, gt_poses):
        ax.scatter(gt_poses[:,0], gt_poses[:,1], gt_poses[:,2], c='gray', marker='o', alpha=0.3)
        for p_map, p_real in zip(map_poses, gt_poses):
            ax.plot([p_map[0], p_real[0]], [p_map[1], p_real[1]], [p_map[2], p_real[2]], c='red', linewidth=0.5, alpha=0.5)

    # Câmeras
    ax.scatter(gt_cam[0], gt_cam[1], gt_cam[2], c='green', marker='*', s=300, label='Câmera GT')
    ax.scatter(est_cam[0], est_cam[1], est_cam[2], c='red', marker='X', s=200, label='Estimada')
    ax.plot(history[:,0], history[:,1], history[:,2], c='blue', linestyle='--', label='Trajetória')

    # Texto Informativo
    final_error = np.linalg.norm(est_cam - gt_cam)
    info_text = (
        f"STATUS: {reason}\n"
        f"ITERAÇÕES: {iterations}\n\n"
        f"POSE FINAL:\nX={est_cam[0]:.3f}, Y={est_cam[1]:.3f}, Z={est_cam[2]:.3f}\n\n"
        f"ERRO FINAL: {final_error:.4f} m"
    )
    # Caixa de texto colorida dependendo do status
    box_color = 'lightgreen' if "Tolerância" in reason else 'salmon'
    
    ax.text2D(0.02, 0.95, info_text, transform=ax.transAxes, fontsize=10, verticalalignment='top',
              bbox=dict(boxstyle='round', facecolor=box_color, alpha=0.9, edgecolor='gray'))

    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.view_init(elev=20, azim=130)
    print(f"[Gráfico] Salvando '{filename}'...")
    plt.savefig(filename)

# =============================================================================
# 4. EXECUÇÃO
# =============================================================================
FILENAME = 'synthetic_ground_truth.yaml'

print(f"--- Processando '{FILENAME}' ---")
real_tags, real_dists, real_cam = load_ground_truth(FILENAME)

initial_guess = np.array([3.0, 3.0, 0.0])

# --- SUA TOLERÂNCIA AQUI ---
# Tente mudar para valores como 1e-3, 1e-4, 1e-6 para ver a diferença
MINHA_TOLERANCIA = 1e-6 

print(f"Tolerância Definida: {MINHA_TOLERANCIA}")

# Cenário 1
print("\n>>> Cenário 1: Mapa Perfeito")
est_1, hist_1, iters_1, reason_1, grad_1 = gradient_descent(
    real_tags, real_dists, initial_guess, 
    lr=0.005, stop_threshold=MINHA_TOLERANCIA
)

print(f"Status:             {reason_1}")
print(f"Iterações:          {iters_1}")
print(f"Gradiente Final:    {grad_1:.8f}")
print(f"Erro Real (GT):     {np.linalg.norm(est_1 - real_cam):.6f} m")

plot_simulation(real_tags, real_tags, real_cam, est_1, hist_1, iters_1, reason_1,
                "Cenário 1: Mapa Perfeito", "res_corrigido_limpo.png")

# Cenário 2
print("\n>>> Cenário 2: Mapa Ruidoso")
np.random.seed(42)
pose_noise = np.random.normal(0, 0.05, size=real_tags.shape)
noisy_map_poses = real_tags + pose_noise

est_2, hist_2, iters_2, reason_2, grad_2 = gradient_descent(
    noisy_map_poses, real_dists, initial_guess, 
    lr=0.005, stop_threshold=MINHA_TOLERANCIA
)

print(f"Status:             {reason_2}")
print(f"Iterações:          {iters_2}")
print(f"Gradiente Final:    {grad_2:.8f}")
print(f"Erro Real (GT):     {np.linalg.norm(est_2 - real_cam):.6f} m")

plot_simulation(noisy_map_poses, real_tags, real_cam, est_2, hist_2, iters_2, reason_2,
                "Cenário 2: Mapa Ruidoso", "res_corrigido_ruidoso.png")

plt.show()
