import yaml
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    # --- 1. CARREGAR DADOS ---
    try:
        with open("tags_poses.yaml", "r") as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print("Erro: Crie o arquivo 'tags_cenario.yaml' primeiro!")
        return

    tags_dict = config['fixed_tags']
    tags_matrix = np.array(list(tags_dict.values()))
    tag_ids = list(tags_dict.keys()) # Para colocar nomes no gráfico

    # --- 2. CONFIGURAÇÃO ---
    true_camera_pos = np.array([0.0, 0.0, 0.0])
    estimated_pos = np.array([-2.0, -1.5, -1.0]) 
    
    # Configuração de Alta Precisão
    NOISE_LEVEL = 0.05  # 5 cm de ruído
    NUM_SAMPLES = 100   # Média de 100 leituras
    learning_rate = 0.01
    MAX_ITERATIONS = 200

    print(f"--- SIMULAÇÃO COMPLETA ---")
    print(f"Tags no cenário: {len(tags_matrix)}")
    print(f"Filtrando ruído de {NOISE_LEVEL*100}cm com {NUM_SAMPLES} amostras...")

    # --- 3. FILTRAGEM DE RUÍDO (Média) ---
    ideal_observations = tags_matrix - true_camera_pos
    accumulated_obs = np.zeros_like(ideal_observations)
    
    for _ in range(NUM_SAMPLES):
        noise = np.random.normal(0, NOISE_LEVEL, tags_matrix.shape)
        accumulated_obs += (ideal_observations + noise)
        
    averaged_observations = accumulated_obs / NUM_SAMPLES

    # --- 4. OTIMIZAÇÃO ---
    path_history = [estimated_pos.copy()]
    
    for i in range(MAX_ITERATIONS):
        predictions = tags_matrix - estimated_pos
        residuals = predictions - averaged_observations
        gradient = -np.sum(residuals, axis=0)
        
        estimated_pos = estimated_pos - (learning_rate * gradient)
        path_history.append(estimated_pos.copy())

    # --- 5. RESULTADOS ---
    final_error = np.linalg.norm(estimated_pos - true_camera_pos)
    print(f"Erro Final: {final_error*100:.3f} cm")

    # --- 6. PLOTAGEM COM APRITAGS ---
    plot_full_scene(tags_matrix, tag_ids, np.array(path_history), true_camera_pos, final_error)

def plot_full_scene(tags, tag_names, path, target, error_val):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # --- A. PLOTAR AS APRILTAGS ---
    # Usamos marker='s' (square) para parecer uma tag
    ax.scatter(tags[:,0], tags[:,1], tags[:,2], c='green', marker='s', s=300, label='AprilTags (Parede)', edgecolors='black')
    
    # Adicionar texto (ID da tag) logo acima de cada quadrado
    for i, pos in enumerate(tags):
        ax.text(pos[0], pos[1], pos[2] + 0.2, tag_names[i], fontsize=9, color='darkgreen', ha='center')
        
        # Desenhar uma linha tracejada da Câmera Final até a Tag (Linha de Visão)
        # Isso ajuda a entender a geometria 3D
        final_cam = path[-1]
        ax.plot([final_cam[0], pos[0]], 
                [final_cam[1], pos[1]], 
                [final_cam[2], pos[2]], 
                color='gray', linestyle='--', alpha=0.3, linewidth=1)

    # --- B. PLOTAR CÂMERA ---
    # Trajetória
    ax.plot(path[:,0], path[:,1], path[:,2], c='blue', linewidth=2, label='Trajetória Otimização')
    
    # Início
    ax.scatter(path[0,0], path[0,1], path[0,2], c='red', s=50, label='Início (Chute)')
    
    # Fim Estimado
    ax.scatter(path[-1,0], path[-1,1], path[-1,2], c='cyan', s=150, edgecolors='black', marker='o', label='Câmera Estimada')
    
    # Fim Real (Alvo)
    ax.scatter(target[0], target[1], target[2], c='gold', marker='*', s=250, label='Câmera Real (0,0,0)')

    # --- C. DETALHES VISUAIS ---
    # Caixa de informações
    info = f"Tags: {len(tags)}\nAmostras: 100\nErro Final: {error_val*100:.2f} cm"
    ax.text2D(0.02, 0.90, info, transform=ax.transAxes, 
              bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

    ax.set_title("Visualização PnP: Câmera vs AprilTags")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    
    # Ajustar a visão para parecer uma sala
    ax.set_xlim(-2.5, 1.0) # Profundidade
    ax.set_ylim(-2.0, 2.0) # Largura
    ax.set_zlim(-1.0, 2.5) # Altura
    
    ax.legend(loc='lower left')
    plt.show()

if __name__ == "__main__":
    main()
