#!/usr/bin/env python3
import numpy as np
import yaml
import os
from scipy.spatial.transform import Rotation as R

def main():
    # --- CONFIGURAÇÃO DO CENÁRIO ---
    RADIUS = 2.0          # Raio do círculo em metros
    HEIGHT = 1.0          # Altura das tags (Z) em metros
    START_ANGLE = -90     # Ângulo inicial em graus (Direita do robo, -90deg em Y)
    END_ANGLE = 90        # Ângulo final em graus (Esquerda do robo, 90deg em Y)
    NUM_TAGS = 10         # Quantas tags distribuir nesse arco
    TAG_FAMILY = "tag36h11"
    START_ID = 0          # ID da primeira tag
    WORLD_FRAME = "world" # Nome do frame pai para as Ground Truth TFs

    # Dicionário final
    ground_truth = {'poses': {}, 'distances': {}}
    
    # Gera ângulos espaçados igualmente
    angles = np.linspace(np.radians(START_ANGLE), np.radians(END_ANGLE), NUM_TAGS)
    
    print(f"Gerando {NUM_TAGS} tags em um arco de raio {RADIUS}m no frame '{WORLD_FRAME}'...")

    positions = {} # Para calculo de distancias depois

    for i, angle_rad in enumerate(angles):
        tag_id = START_ID + i
        tag_name = f"{TAG_FAMILY}:{tag_id}"
        gt_tag_frame = f"gt_{tag_name}" # Nome do frame para a Ground Truth TF

        # 1. POSIÇÃO (Coordenadas Polares -> Cartesianas)
        # No frame do mundo (Robo na origem, X frente, Y esquerda, Z cima)
        x = RADIUS * np.cos(angle_rad)
        y = RADIUS * np.sin(angle_rad)
        z = HEIGHT

        # 2. ORIENTAÇÃO (Olhando para o centro - origem do world frame)
        # O eixo Z da tag (normal da face) deve apontar para a origem [0,0,0]
        
        # Vetor da posição da tag no mundo
        pos_vec = np.array([x, y, z])
        
        # Vetor do eixo Z da tag (aponta da tag para a origem, mas na altura da tag)
        # Para manter a tag vertical e olhando para o centro do círculo no chão
        target_point = np.array([0.0, 0.0, HEIGHT]) # Ponto no centro do círculo na mesma altura da tag
        z_axis_tag = target_point - pos_vec
        z_axis_tag = z_axis_tag / np.linalg.norm(z_axis_tag) # Normalizado

        # Eixo Y da tag (aponta para baixo na face da tag).
        # Para que a tag esteja "em pé", o eixo Y da tag deve ser alinhado com o Z global (para cima/baixo).
        # Assumimos que o "top" da tag (direção -Y do frame da tag) aponta para o Z positivo do mundo
        # Ou seja, o eixo Y do frame da tag deve ser aproximadamente -Z do mundo
        
        # O Eixo X da tag (para a "direita" da face da tag) deve ser ortogonal a Z_tag e Y_tag.
        # Vamos definir o eixo Y da tag como sendo 'down' em relação ao world frame
        # e então ajustar o X da tag para ser ortogonal a Y_tag e Z_tag.
        
        # Tentativa de definir a orientação com Z_axis_tag apontando para o centro
        # E o Y_axis_tag (para baixo na tag) aproximadamente na direção Z_world (para cima)
        
        # Definir 'up' no mundo. Se a tag está na vertical, o 'top' dela aponta para o +Z do mundo
        # A convenção da AprilTag é Z out, Y down, X right.
        # Então, o eixo Y da tag (-y_tag) deve ser alinhado com o z_world.
        
        # Eixo Z da tag aponta para o centro
        vec_z_tag_world = z_axis_tag # Este é o Z_COL da matriz de rotação

        # Eixo Y da tag (aponta para baixo na imagem da tag) deve ser o Z do mundo (para cima)
        # ou seja, o "topo" da tag é para cima no mundo.
        vec_y_tag_world = np.array([0.0, 0.0, 1.0]) # Este é o Y_COL da matriz de rotação

        # Se vec_z_tag_world e vec_y_tag_world forem paralelos, isso falha (tag olhando para cima/baixo)
        # Como estamos num círculo no plano XY, vec_z_tag_world sempre terá componente XY significativa.
        
        # Corrigindo para ser robusto (Gram-Schmidt-like process for orthonormal basis)
        # O eixo X da tag deve ser ortogonal a Y_tag_world e Z_tag_world
        vec_x_tag_world = np.cross(vec_y_tag_world, vec_z_tag_world)
        vec_x_tag_world = vec_x_tag_world / np.linalg.norm(vec_x_tag_world) # Normaliza
        
        # Recalcula o Y para garantir ortogonalidade perfeita
        vec_y_tag_world = np.cross(vec_z_tag_world, vec_x_tag_world)
        vec_y_tag_world = vec_y_tag_world / np.linalg.norm(vec_y_tag_world) # Normaliza

        # Matriz de Rotação [X_col | Y_col | Z_col]
        R_mat = np.column_stack((vec_x_tag_world, vec_y_tag_world, vec_z_tag_world))
        
        # Converte para Quaternion (x, y, z, w)
        r = R.from_matrix(R_mat)
        qx, qy, qz, qw = r.as_quat()

        # Salva no dict
        ground_truth['poses'][tag_id] = {
            'name': tag_name,
            'gt_frame_id': gt_tag_frame, # Novo campo para o nome do frame da Ground Truth TF
            'parent_frame_id': WORLD_FRAME, # Novo campo para o frame pai
            'pose': {
                'position': {'x': float(x), 'y': float(y), 'z': float(z)},
                'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
            }
        }
        positions[tag_name] = pos_vec
        
        print(f"  Tag {tag_name} ({gt_tag_frame}): Pos({x:.2f}, {y:.2f}, {z:.2f}) | Angulo {np.degrees(angle_rad):.1f} deg")

    # --- CALCULA DISTANCIAS (Inter-Tag) ---
    # Util para validação relativa (sem depender do zero absoluto do mundo)
    dist_dict = {}
    names = list(positions.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            n1, n2 = names[i], names[j]
            d = np.linalg.norm(positions[n1] - positions[n2])
            dist_dict[f"{n1}_to_{n2}"] = float(d)
    
    ground_truth['distances'] = dist_dict
    ground_truth['world_frame'] = WORLD_FRAME # Adiciona o nome do frame do mundo ao YAML

    # Salva Arquivo
    output_file = 'synthetic_ground_truth.yaml'
    with open(output_file, 'w') as f:
        yaml.dump(ground_truth, f, default_flow_style=False, indent=2)
    
    print(f"\nArquivo gerado: {os.path.abspath(output_file)}")

if __name__ == "__main__":
    # Instala scipy se nao tiver (usado para quaternion math robusto)
    try:
        import scipy
        from scipy.spatial.transform import Rotation as R # Para garantir que a importação funciona antes de main()
    except ImportError:
        print("Instalando scipy...")
        # Usa pip para instalar
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "scipy"])
        import scipy
        from scipy.spatial.transform import Rotation as R
        
    main()