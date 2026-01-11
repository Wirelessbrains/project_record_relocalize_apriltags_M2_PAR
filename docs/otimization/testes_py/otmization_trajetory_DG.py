import yaml
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    filename = "sensor_data.yaml"
    
    # 1. CARREGAR ARQUIVO
    try:
        with open(filename, "r") as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print("Erro: Rode o gerador primeiro.")
        return

    # 2. LER O MAPA (FIXED TAGS)
    if 'fixed_tags' not in data:
        print("Erro: O YAML não contém 'fixed_tags'.")
        return

    world_tags = {}
    for tid, pos in data['fixed_tags'].items():
        world_tags[tid] = np.array([pos['x'], pos['y'], pos['z']])

    print(f"Mapa carregado: {len(world_tags)} tags conhecidas.")

    # 3. PROCESSAR FRAMES
    total_frames = data['metadata']['total_frames']
    path_estimated = []
    
    # Chute inicial: Começamos na origem (0,0,0) e deixamos a matemática nos levar
    current_camera_pose = np.array([0.0, 0.0, 0.0])

    for i in range(total_frames):
        observations = []
        
        # Coletar o que o sensor viu neste frame
        for tag_id, logs in data['tags_log'].items():
            if tag_id in world_tags:
                p = logs[i]['pose']
                # Esta é a pose RELATIVA (Tag vista da Camera)
                vec_relativo = np.array([p['x'], p['y'], p['z']])
                
                # Guardar par: (Onde a tag é no mundo, Onde ela está p/ camera)
                observations.append((world_tags[tag_id], vec_relativo))

        # OTIMIZAR: Descobrir onde está a câmera
        current_camera_pose = solve_camera_position(observations, current_camera_pose)
        path_estimated.append(current_camera_pose)

    # 4. PLOTAR
    plot_complete_scene(world_tags, np.array(path_estimated))

def solve_camera_position(obs, guess):
    """
    Resolve: Camera_Mundo = Tag_Mundo - Leitura_Sensor
    Usamos média ou gradiente se tivermos várias tags.
    """
    # Se tivermos apenas 1 tag, a conta é direta:
    # Cam = Tag_Mundo - Leitura
    # Com várias tags, fazemos o Gradiente Descendente para achar o melhor ajuste
    
    estimated = guess.copy()
    lr = 0.1
    for _ in range(50): # 50 iterações
        t_world = np.array([o[0] for o in obs])   # Posições Reais
        t_sensor = np.array([o[1] for o in obs])  # Leituras Relativas
        
        # A equação fundamental: Leitura = Real - Camera
        # Predição da Leitura = Real - Camera_Estimada
        pred_sensor = t_world - estimated
        
        # Erro = Predição - Leitura Real
        residuals = pred_sensor - t_sensor
        
        # Atualizar estimativa da câmera
        grad = -np.sum(residuals, axis=0)
        estimated -= (lr * grad)
        
    return estimated

def plot_complete_scene(tags, path):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Tags na Parede (Y=0)
    tm = np.array(list(tags.values()))
    ax.scatter(tm[:,0], tm[:,1], tm[:,2], c='green', s=200, marker='s', label='Tags (Mundo)', edgecolors='k')

    # Trajetória da Câmera Recuperada
    ax.plot(path[:,0], path[:,1], path[:,2], c='blue', linestyle='--', linewidth=2, label='Câmera Recuperada')
    
    # Pontos da Câmera
    ax.scatter(path[:,0], path[:,1], path[:,2], c=range(len(path)), cmap='cool', s=50)

    ax.set_title("Reconstrução: Tags (Y=0) vs Câmera (Móvel)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y (Profundidade)")
    ax.set_zlabel("Z")
    
    # Ajuste para ver a separação
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-3.0, 0.5)
    ax.set_zlim(-1.0, 1.5)
    ax.view_init(elev=30, azim=-60)
    
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
