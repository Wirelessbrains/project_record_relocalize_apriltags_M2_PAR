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

    # --- 2. CONFIGURAÇÃO ---
    true_camera_pos = np.array([0.0, 0.0, 0.0])
    estimated_pos = np.array([-2.0, -1.5, -1.0]) # Começa longe
    
    # META: Erro menor que 1 cm (0.01m)
    # Vamos ser ambiciosos: meta de 1 milímetro (0.001)
    TARGET_ERROR = 0.001 
    MAX_ITERATIONS = 2000 # Trava de segurança para não rodar para sempre
    
    learning_rate = 0.01

    path_history = [estimated_pos.copy()]

    print(f"--- INICIANDO OTIMIZAÇÃO DE ALTA PRECISÃO ---")
    print(f"Meta de Erro: < {TARGET_ERROR*100} cm")
    
    current_error = 999.0
    iteration = 0

    # --- 3. LOOP INTELIGENTE (Roda até atingir a precisão) ---
    while current_error > TARGET_ERROR and iteration < MAX_ITERATIONS:
        
        # A. Matemática do Gradiente
        observations = tags_matrix - true_camera_pos
        predictions = tags_matrix - estimated_pos
        residuals = predictions - observations
        gradient = -np.sum(residuals, axis=0)

        # B. Atualizar Posição
        estimated_pos = estimated_pos - (learning_rate * gradient)
        path_history.append(estimated_pos.copy())

        # C. Calcular o Erro Atual (Distância Euclidiana)
        dist_vector = estimated_pos - true_camera_pos
        current_error = np.linalg.norm(dist_vector)

        # Log a cada 50 iterações para não sujar o terminal
        if iteration % 50 == 0:
            print(f"Iter {iteration}: Erro atual = {current_error:.5f} m")

        iteration += 1

    # --- 4. RESULTADOS ---
    final_pose = np.round(estimated_pos, 5)
    
    print(f"\n--- SUCESSO! ---")
    print(f"Iterações necessárias: {iteration}")
    print(f"Pose Real:      {true_camera_pos}")
    print(f"Pose Estimada:  {final_pose}")
    print(f"-----------------------------")
    # Convertendo para cm para facilitar a leitura
    print(f"ERRO FINAL: {current_error*100:.4f} cm") 
    
    if current_error < 0.01:
        print(">> OBJETIVO ATINGIDO (Erro < 1cm)")
    else:
        print(">> AVISO: Máximo de iterações atingido antes da convergência.")

    # --- 5. PLOTAR ---
    plot_results(tags_matrix, np.array(path_history), true_camera_pos, current_error)

def plot_results(tags, path, target, final_error):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Tags
    ax.scatter(tags[:,0], tags[:,1], tags[:,2], c='green', marker='s', s=100, label='Tags')
    # Trajetória
    ax.plot(path[:,0], path[:,1], path[:,2], c='blue', linewidth=1, label='Caminho')
    # Pontos
    ax.scatter(path[0,0], path[0,1], path[0,2], c='red', label='Início')
    ax.scatter(path[-1,0], path[-1,1], path[-1,2], c='cyan', s=100, marker='o', label='Fim (Estimado)')
    ax.scatter(target[0], target[1], target[2], c='gold', marker='*', s=150, label='Alvo (Real)')

    # Caixa de Texto com o erro em cm
    info_text = f"Erro Final: {final_error*100:.3f} cm"
    ax.text2D(0.05, 0.95, info_text, transform=ax.transAxes, 
              bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8), fontsize=12)

    ax.set_title("Otimização de Alta Precisão")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    
    # Zoom automático perto do alvo para ver a precisão
    # ax.set_xlim(-0.5, 0.5); ax.set_ylim(-0.5, 0.5); ax.set_zlim(-0.5, 0.5)

    plt.show()

if __name__ == "__main__":
    main()
