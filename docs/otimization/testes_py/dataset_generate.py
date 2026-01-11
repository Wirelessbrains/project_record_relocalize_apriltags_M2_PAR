import yaml
import math
import numpy as np

def main():
    # --- 1. CONFIGURAÇÃO DO MAPA (Mundo Global) ---
    # Tags coladas na parede Y=0. Variam em X e Z.
    fixed_tags = {
        'tag_0': {'x': -0.5, 'y': 0.0, 'z':  0.5},
        'tag_1': {'x':  0.5, 'y': 0.0, 'z':  0.5},
        'tag_2': {'x':  0.0, 'y': 0.0, 'z': -0.5},
        'tag_3': {'x':  0.0, 'y': 0.0, 'z':  0.0}
    }

    # --- 2. CONFIGURAÇÃO DA CÂMERA (Simulação) ---
    NUM_FRAMES = 30
    RAIO = 0.5
    CENTRO_Y = -2.0 # O robô está a 2 metros da parede

    dataset = {
        'metadata': {
            'descricao': 'Log de Sensor: Poses das Tags em relacao a Camera',
            'total_frames': NUM_FRAMES
        },
        # Salvamos o mapa no arquivo para o otimizador usar depois
        'fixed_tags': fixed_tags, 
        'tags_log': {}
    }

    # Inicializar logs
    for tag_id in fixed_tags:
        dataset['tags_log'][tag_id] = []

    print("Gerando leituras do sensor (Tag relativa à Câmera)...")

    for i in range(NUM_FRAMES):
        # Movimento real da câmera (Onde ela está de verdade)
        angle = (2 * math.pi * i) / NUM_FRAMES
        cam_global_x = RAIO * math.cos(angle)
        cam_global_y = CENTRO_Y + (RAIO * math.sin(angle))
        cam_global_z = 0.0

        for tag_id, tag_world_pos in fixed_tags.items():
            # --- A MATEMÁTICA DO SENSOR ---
            # O que a câmera vê = Posição_Mundo_Tag - Posição_Mundo_Camera
            # Ex: Se Tag está em 0 e Camera em -2, a Tag está em +2 relativo à câmera.
            
            obs_x = tag_world_pos['x'] - cam_global_x
            obs_y = tag_world_pos['y'] - cam_global_y
            obs_z = tag_world_pos['z'] - cam_global_z

            entry = {
                'frame_seq': i,
                'pose': {
                    'x': float(f"{obs_x:.4f}"),
                    'y': float(f"{obs_y:.4f}"),
                    'z': float(f"{obs_z:.4f}")
                }
            }
            dataset['tags_log'][tag_id].append(entry)

    with open("sensor_data.yaml", 'w') as f:
        yaml.dump(dataset, f, sort_keys=False)
    print("Arquivo 'sensor_data.yaml' gerado.")

if __name__ == "__main__":
    main()
