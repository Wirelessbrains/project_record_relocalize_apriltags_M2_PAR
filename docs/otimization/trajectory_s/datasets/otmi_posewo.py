import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import pandas as pd
import sys

# ==============================================================================
# 1. FUNÇÕES AUXILIARES DE ÁLGEBRA LINEAR
# ==============================================================================
def pose_to_matrix(pos_dict, rot_dict):
    """Converte dados do YAML (pos dict, rot dict) para Matriz 4x4 Homogênea."""
    # Posição
    tx = pos_dict['x']
    ty = pos_dict['y']
    tz = pos_dict['z']
    
    # Rotação (Quatérnio)
    qx = rot_dict['x']
    qy = rot_dict['y']
    qz = rot_dict['z']
    qw = rot_dict['w']
    
    # Criar matriz 4x4
    # T = [ R  t ]
    #     [ 0  1 ]
    r_mat = R.from_quat([qx, qy, qz, qw]).as_matrix()
    t_vec = np.array([tx, ty, tz])
    
    T = np.eye(4)
    T[:3, :3] = r_mat
    T[:3, 3] = t_vec
    return T

def average_matrices(matrices):
    """
    Calcula a média de uma lista de matrizes de transformação 4x4.
    - Posição: Média aritmética simples.
    - Rotação: Média via Quatérnios (para manter propriedades de rotação).
    """
    if not matrices:
        return None
    
    # Média das Translações
    ts = np.array([M[:3, 3] for M in matrices])
    t_avg = np.mean(ts, axis=0)
    
    # Média das Rotações
    # Converte todas para quatérnio, tira média e normaliza
    quats = np.array([R.from_matrix(M[:3, :3]).as_quat() for M in matrices])
    # Truque simples para média de quatérnios próximos: média direta + normalização
    # (Para casos mais complexos precisaria de autovetores, mas aqui serve)
    
    # Garantir continuidade (w positivo)
    if len(quats) > 1:
        for i in range(1, len(quats)):
            if np.dot(quats[0], quats[i]) < 0:
                quats[i] = -quats[i]
                
    q_avg = np.mean(quats, axis=0)
    q_avg = q_avg / np.linalg.norm(q_avg) # Normalizar
    
    T_avg = np.eye(4)
    T_avg[:3, :3] = R.from_quat(q_avg).as_matrix()
    T_avg[:3, 3] = t_avg
    return T_avg

# ==============================================================================
# 2. CARREGAMENTO
# ==============================================================================
filename = 'circle_dense_truth.yaml'
print(f"Lendo {filename}...")

try:
    with open(filename, 'r') as f:
        dataset = yaml.safe_load(f)
except Exception as e:
    print(f"Erro: {e}")
    sys.exit(1)

frames = dataset['frames']
num_frames = len(frames)

# ==============================================================================
# 3. ALGORITMO: MAPA E TRAJETÓRIA BASEADOS EM GRAFOS DE POSE
# ==============================================================================
# Estruturas de Dados
# known_tags[tag_id] = Matriz 4x4 (Pose da Tag no Mundo)
known_tags = {} 
trajectory_matrices = [None] * num_frames

print("Construindo trajetória usando Orientação das Tags (6-DoF)...")

# --- Inicialização (Frame 0) ---
# Definimos o Frame 0 como a Origem do Mundo (Camera = Identity)
cam_T_world_init = np.eye(4)
trajectory_matrices[0] = cam_T_world_init

# Mapear tags iniciais
det0 = frames[0].get('detections') or frames[0].get('tags') or {}
for tag_id, data in det0.items():
    # Matriz da Tag em relação à Câmera (Medição)
    tag_T_cam = pose_to_matrix(data['position'], data['orientation'])
    
    # Pose da Tag no Mundo = Pose_Cam_Mundo * Pose_Tag_Cam
    # Como Cam_Mundo é Identity -> Tag_Mundo = Tag_Cam
    tag_T_world = cam_T_world_init @ tag_T_cam
    known_tags[tag_id] = tag_T_world

# --- Loop Frame a Frame ---
for i in range(1, num_frames):
    frame = frames[i]
    detections = frame.get('detections') or frame.get('tags') or {}
    
    candidate_cam_poses = []
    
    # Passo A: Onde está a câmera? (Baseado nas Tags Conhecidas)
    for tag_id, data in detections.items():
        if tag_id in known_tags:
            # Medição atual: Onde está a Tag em relação à Câmera AGORA
            tag_T_cam_meas = pose_to_matrix(data['position'], data['orientation'])
            
            # Sabemos onde a Tag está no Mundo (known_tags[tag_id])
            # Queremos: Cam_T_World
            # Relação: Tag_T_World = Cam_T_World * Tag_T_Cam
            # Logo:    Cam_T_World = Tag_T_World * inv(Tag_T_Cam)
            
            tag_T_world_known = known_tags[tag_id]
            cam_T_world_candidate = tag_T_world_known @ np.linalg.inv(tag_T_cam_meas)
            
            candidate_cam_poses.append(cam_T_world_candidate)
            
    # Passo B: Atualizar Pose da Câmera
    if candidate_cam_poses:
        # Se vimos várias tags conhecidas, fazemos a média para reduzir ruído
        current_cam_T_world = average_matrices(candidate_cam_poses)
        trajectory_matrices[i] = current_cam_T_world
        
        # Passo C: Adicionar NOVAS tags ao mapa
        for tag_id, data in detections.items():
            if tag_id not in known_tags:
                tag_T_cam_meas = pose_to_matrix(data['position'], data['orientation'])
                
                # Nova Tag no Mundo = Cam_T_World (Atual) * Tag_T_Cam (Medição)
                new_tag_T_world = current_cam_T_world @ tag_T_cam_meas
                known_tags[tag_id] = new_tag_T_world
    else:
        # Perda de tracking (nenhuma tag conhecida visível)
        trajectory_matrices[i] = None

# ==============================================================================
# 4. INTERPOLAÇÃO E EXTRAÇÃO DE DADOS
# ==============================================================================
print("Interpolando trajetórias...")

# Extrair posições (t) e rotações (q)
positions = []
rotations = []
indices_valid = []

for i, M in enumerate(trajectory_matrices):
    if M is not None:
        positions.append(M[:3, 3])
        rotations.append(R.from_matrix(M[:3, :3]))
        indices_valid.append(i)
    else:
        positions.append([np.nan, np.nan, np.nan])
        rotations.append(None)

positions = np.array(positions)

# Interpolando Posição (Pandas é ótimo para isso)
df_pos = pd.DataFrame(positions, columns=['x', 'y', 'z'])
df_pos = df_pos.interpolate(method='linear', limit_direction='both')
final_positions = df_pos.to_numpy()

# ==============================================================================
# 5. VISUALIZAÇÃO
# ==============================================================================
print(f"Total de Tags Mapeadas: {len(known_tags)}")
tag_positions = np.array([M[:3, 3] for M in known_tags.values()])

plt.figure(figsize=(10, 10))

# Plota trajetória da Câmera
plt.plot(final_positions[:, 0], final_positions[:, 1], '-', color='blue', linewidth=2, label='Trajetória Câmera')

# Plota onde houve detecção real (frames válidos)
valid_x = final_positions[indices_valid, 0]
valid_y = final_positions[indices_valid, 1]
plt.scatter(valid_x, valid_y, s=10, c='cyan', alpha=0.5, label='Frames com Tracking')

# Plota as Tags Mapeadas
if len(tag_positions) > 0:
    plt.scatter(tag_positions[:, 0], tag_positions[:, 1], c='red', marker='x', s=100, label='Tags (Mundo)')

# Decoração
plt.title("Visual SLAM 6-DoF (Usa Orientação das Tags)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis('equal') # Fundamental para não distorcer o círculo
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend()
plt.tight_layout()

plt.savefig('slam_6dof_corrigido.png')
print("Gráfico salvo como 'slam_6dof_corrigido.png'")
plt.show()
