import yaml
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    filename = "sensor_data.yaml"
    
    # --- CONFIGURAÇÃO ---
    NOISE_LEVEL = 0.05  # 5cm de ruído
    
    # 1. CARREGAR ARQUIVO
    try:
        with open(filename, "r") as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print("Erro: Rode o gerador 'gerar_sensor_real.py' primeiro.")
        return

    if 'fixed_tags' not in data: return
    world_tags = {tid: np.array([p['x'], p['y'], p['z']]) for tid, p in data['fixed_tags'].items()}

    # 2. PROCESSAMENTO DUPLO
    total_frames = data['metadata']['total_frames']
    
    path_ideal = []
    path_noisy = []
    
    guess_ideal = np.array([0.0, -2.0, 0.0])
    guess_noisy = np.array([0.0, -2.0, 0.0])

    print(f"Calculando erro para {total_frames} frames (Ruído: {NOISE_LEVEL*100}cm)...")

    for i in range(total_frames):
        obs_ideal = []
        obs_noisy = []
        
        for tag_id, logs in data['tags_log'].items():
            if tag_id in world_tags:
                p = logs[i]['pose']
                vec_perfeito = np.array([p['x'], p['y'], p['z']])
                
                # Adicionar Ruído
                ruido = np.random.normal(0, NOISE_LEVEL, 3)
                vec_ruidoso = vec_perfeito + ruido
                
                obs_ideal.append((world_tags[tag_id], vec_perfeito))
                obs_noisy.append((world_tags[tag_id], vec_ruidoso))

        if obs_ideal:
            guess_ideal = solve_camera_position(obs_ideal, guess_ideal)
            path_ideal.append(guess_ideal)
            
        if obs_noisy:
            guess_noisy = solve_camera_position(obs_noisy, guess_noisy)
            path_noisy.append(guess_noisy)

    # --- 3. CÁLCULO DO ERRO ---
    path_ideal_np = np.array(path_ideal)
    path_noisy_np = np.array(path_noisy)
    
    # Distância Euclidiana entre Ideal e Ruidoso (Ponto a Ponto)
    # axis=1 calcula a norma linha por linha
    errors_abs = np.linalg.norm(path_noisy_np - path_ideal_np, axis=1)
    
    # Estatísticas
    avg_error_m = np.mean(errors_abs)
    max_error_m = np.max(errors_abs)
    
    # Erro percentual relativo à distância da parede (Origem)
    # Distância real do robô até a origem (Y=0)
    dist_to_origin = np.linalg.norm(path_ideal_np, axis=1)
    errors_pct = (errors_abs / dist_to_origin) * 100
    avg_pct = np.mean(errors_pct)

    print(f"\n=== RELATÓRIO DE ERROS ===")
    print(f"Erro Médio:   {avg_error_m*100:.2f} cm")
    print(f"Erro Máximo:  {max_error_m*100:.2f} cm")
    print(f"Erro Percep.: {avg_pct:.2f} %")

    # 4. PLOTAR COM INFO DE ERRO
    plot_comparison(world_tags, path_ideal_np, path_noisy_np, avg_error_m, avg_pct)

def solve_camera_position(obs, guess):
    estimated = guess.copy()
    lr = 0.1
    for _ in range(50): 
        t_w = np.array([o[0] for o in obs])   
        t_s = np.array([o[1] for o in obs])  
        pred_sensor = t_w - estimated
        residuals = pred_sensor - t_s
        grad = -np.sum(residuals, axis=0)
        if len(obs) > 0: grad = grad / len(obs)
        estimated -= (lr * grad)
    return estimated

def plot_comparison(tags, path_ideal, path_noisy, avg_err, avg_pct):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Tags
    tm = np.array(list(tags.values()))
    ax.scatter(tm[:,0], tm[:,1], tm[:,2], c='green', s=150, marker='s', label='Tags (Y=0)', edgecolors='k')

    # Ideal (Círculos)
    ax.scatter(path_ideal[:,0], path_ideal[:,1], path_ideal[:,2], 
               c='cyan', marker='o', s=60, label='Ideal (Sem Ruído)', alpha=0.4, edgecolors='none')

    # Ruidoso (Pontilhado)
    ax.plot(path_noisy[:,0], path_noisy[:,1], path_noisy[:,2], 
            c='red', linestyle=':', linewidth=2, label='Estimado (Com Ruído)')

    # --- CAIXA DE TEXTO COM O ERRO ---
    info_text = (
        f"ESTATÍSTICAS DE ERRO\n"
        f"--------------------\n"
        f"Ruído Sensor: 5.0 cm\n"
        f"Erro Médio:   {avg_err*100:.2f} cm\n"
        f"Erro %:       {avg_pct:.2f} %"
    )
    # Posiciona no canto superior esquerdo (coordenadas 2D da janela)
    ax.text2D(0.05, 0.95, info_text, transform=ax.transAxes, 
              bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
              fontsize=11, family='monospace')

    ax.set_title("Análise de Precisão: Ideal vs Real")
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_xlim(-1.5, 1.5); ax.set_ylim(-3.0, 0.5); ax.set_zlim(-1.0, 1.5)
    ax.view_init(elev=30, azim=-60)
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
