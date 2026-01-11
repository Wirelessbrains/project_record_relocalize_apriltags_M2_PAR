import numpy as np
import matplotlib.pyplot as plt
import re
import sys

# =============================================================================
# 1. PARSER E LEITURA
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
# 2. OTIMIZAÇÃO VETORIZADA (BATCH)
# =============================================================================
def gradient_descent_batch(dataset_poses, measured_dists, initial_guess, lr=0.01, max_iter=5000, stop_threshold=1e-5):
    """
    Processamento paralelo de 30 amostras usando tensores NumPy.
    Sem loops 'for' externos.
    """
    num_samples = dataset_poses.shape[0] 
    
    # Expande chute inicial para (30, 3)
    est_pos = np.tile(initial_guess, (num_samples, 1))
    history = [est_pos.copy()]
    
    for i in range(max_iter):
        # Broadcasting para cálculo de diferenças e distâncias
        est_expanded = est_pos[:, np.newaxis, :] 
        diff = est_expanded - dataset_poses 
        
        calc_dists = np.linalg.norm(diff, axis=2)
        
        safe_dists = calc_dists.copy()
        safe_dists[safe_dists < 1e-9] = 1e-9
        
        error = calc_dists - measured_dists 
        
        # Cálculo do gradiente vetorizado
        term = (error[:, :, np.newaxis] * (diff / safe_dists[:, :, np.newaxis]))
        grad = 2 * np.sum(term, axis=1)
        
        # Atualização
        est_pos = est_pos - (lr * grad)
        history.append(est_pos.copy())
        
        # Critério de parada (se a pior amostra convergiu)
        max_grad_norm = np.max(np.linalg.norm(grad, axis=1))
        if max_grad_norm < stop_threshold:
            print(f"    -> Batch convergiu na iteração {i+1}")
            break
            
    return est_pos, np.array(history)

# =============================================================================
# 3. VISUALIZAÇÃO (CORRIGIDA: MÉDIA FORTE / AMOSTRAS TRANSPARENTES)
# =============================================================================
def plot_batch_results(tags_reference, gt_cam, final_estimates, history_tensor, title, filename):
    """
    final_estimates: (30, 3)
    history_tensor: (Iterações, 30, 3)
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 1. Tags (Cenário)
    ax.scatter(tags_reference[:,0], tags_reference[:,1], tags_reference[:,2], 
               c='black', marker='s', s=30, label='Tags (Ref.)')

    # 2. Câmera Verdadeira (GT)
    ax.scatter(gt_cam[0], gt_cam[1], gt_cam[2], 
               c='lime', marker='*', s=400, label='GT (Real)', zorder=20, edgecolors='black')

    # 3. Plotar Amostras Individuais (MUITO TRANSPARENTES)
    num_samples = final_estimates.shape[0]
    
    # Usamos label="_nolegend_" para não poluir a legenda, exceto o primeiro
    for k in range(num_samples):
        traj = history_tensor[:, k, :]
        
        # Legenda genérica para o grupo
        lbl = 'Amostras (Incerteza)' if k == 0 else "_nolegend_"
        
        # Linha fina e transparente (Azul Claro)
        ax.plot(traj[:,0], traj[:,1], traj[:,2], 
                c='cyan', alpha=0.15, linewidth=0.8)
        
        # Ponto final transparente (Laranja Claro)
        ax.scatter(final_estimates[k, 0], final_estimates[k, 1], final_estimates[k, 2], 
                   c='orange', marker='.', s=50, alpha=0.2, label=lbl)

    # 4. CALCULAR E PLOTAR A MÉDIA (COR FORTE)
    # Média ao longo do eixo das amostras (axis=1 para historico, axis=0 para final)
    mean_history = np.mean(history_tensor, axis=1) # (Iterações, 3)
    mean_final = np.mean(final_estimates, axis=0)  # (3,)

    # Trajetória Média (Azul Escuro Forte)
    ax.plot(mean_history[:,0], mean_history[:,1], mean_history[:,2], 
            c='blue', linewidth=3, alpha=1.0, label='Trajetória Média')
    
    # Resultado Final Médio (X Vermelho Forte)
    ax.scatter(mean_final[0], mean_final[1], mean_final[2], 
               c='red', marker='X', s=300, alpha=1.0, label='Média Final', zorder=15, edgecolors='white')

    # Linha visual conectando Média ao GT para ver o erro
    ax.plot([gt_cam[0], mean_final[0]], [gt_cam[1], mean_final[1]], [gt_cam[2], mean_final[2]], 
            c='red', linestyle=':', linewidth=1.5)

    # Estatísticas na Caixa de Texto
    errors = np.linalg.norm(final_estimates - gt_cam, axis=1)
    mean_error = np.mean(errors)
    std_error = np.std(errors)
    
    info_text = (
        f"BATCH: {num_samples} amostras\n"
        f"ERRO DA MÉDIA: {np.linalg.norm(mean_final - gt_cam):.4f} m\n"
        f"DISPERSÃO (STD): {std_error:.4f} m\n"
        f"-----------------\n"
        f"Posição Média:\n"
        f"X={mean_final[0]:.3f}\n"
        f"Y={mean_final[1]:.3f}\n"
        f"Z={mean_final[2]:.3f}"
    )
    
    ax.text2D(0.02, 0.95, info_text, transform=ax.transAxes, fontsize=10, verticalalignment='top',
              bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='black'))

    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=20, azim=130)
    
    print(f"[Gráfico] Salvando '{filename}'...")
    plt.savefig(filename)

# =============================================================================
# 4. EXECUÇÃO
# =============================================================================
if __name__ == "__main__":
    FILENAME = 'synthetic_ground_truth.yaml'
    print(f"--- Lendo '{FILENAME}' ---")
    
    gt_tags, gt_dists, gt_cam = load_ground_truth(FILENAME)
    
    NUM_SAMPLES = 30
    MINHA_TOLERANCIA = 1e-6
    INITIAL_GUESS = np.array([3.0, 3.0, 0.0])
    
    print(f"Configurando Batch Size: {NUM_SAMPLES}")

    # A. PREPARAÇÃO DOS DATASETS (Vetorizado)
    # Shape: (30, N_tags, 3)
    dataset_clean = np.tile(gt_tags[np.newaxis, :, :], (NUM_SAMPLES, 1, 1))

    np.random.seed(42)
    # Ruído aumentado levemente para visualizar melhor a dispersão vs média
    noise = np.random.normal(0, 0.20, size=dataset_clean.shape) 
    dataset_noisy = dataset_clean + noise

    # B. PROCESSAMENTO (Vectorized Gradient Descent)
    print("\n>>> Otimizando Batch 1 (Limpo)...")
    final_clean, hist_clean = gradient_descent_batch(
        dataset_clean, gt_dists, INITIAL_GUESS, stop_threshold=MINHA_TOLERANCIA
    )

    print("\n>>> Otimizando Batch 2 (Ruidoso)...")
    final_noisy, hist_noisy = gradient_descent_batch(
        dataset_noisy, gt_dists, INITIAL_GUESS, stop_threshold=MINHA_TOLERANCIA
    )

    # C. PLOTAGEM
    plot_batch_results(
        dataset_clean[0], 
        gt_cam, 
        final_clean, 
        hist_clean,
        f"Cenário 1: Mapa Perfeito (Convergência Absoluta)", 
        "res_final_limpo.png"
    )

    plot_batch_results(
        dataset_noisy[0], 
        gt_cam, 
        final_noisy, 
        hist_noisy,
        f"Cenário 2: Mapa Ruidoso (Média em Destaque)", 
        "res_final_ruidoso.png"
    )

    plt.show()
