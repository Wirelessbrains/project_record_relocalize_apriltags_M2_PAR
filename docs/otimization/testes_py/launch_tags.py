import os
import sys
import yaml
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # 1. Arquivo de DADOS (Onde estão as coordenadas)
    # Estamos usando o mesmo nome que aparece na sua imagem
    yaml_file_path = os.path.join(script_dir, 'tags_rviz.yaml')
    
    print(f"--- Lendo coordenadas de: {yaml_file_path} ---")

    nodes_to_start = []

    # Tenta ler o YAML
    try:
        with open(yaml_file_path, 'r') as f:
            data = yaml.safe_load(f)
            
            # O seu arquivo começa com "tags:", então buscamos essa chave
            tags = data.get('tags', {})
            
            if not tags:
                print("AVISO: Chave 'tags' não encontrada ou vazia no arquivo.")

    except FileNotFoundError:
        print(f"ERRO: Arquivo {yaml_file_path} não encontrado.")
        return LaunchDescription([])
    except Exception as e:
        print(f"ERRO ao ler YAML: {e}")
        return LaunchDescription([])

    # Cria um publicador para cada tag
    for tag_name, params in tags.items():
        # Verifica se 'params' é um dicionário (tem chaves x, y, z...)
        if isinstance(params, dict):
            # Pega os valores. Se não existir, usa 0.0 como padrão
            x = str(params.get('x', 0.0))
            y = str(params.get('y', 0.0))
            z = str(params.get('z', 0.0))
            
            # Procura por yaw, pitch, roll OU as abreviações r, p, y
            # Nota: No seu yaml, use 'yaw' para não confundir com eixo 'y'
            yaw = str(params.get('yaw', params.get('y', 0.0))) 
            pitch = str(params.get('pitch', params.get('p', 0.0)))
            roll = str(params.get('roll', params.get('r', 0.0)))

            node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'tf_pub_{tag_name}',
                # x y z yaw pitch roll frame_pai frame_filho
                arguments=[x, y, z, yaw, pitch, roll, 'map', tag_name],
                output='screen'
            )
            nodes_to_start.append(node)
            print(f" + Tag carregada: {tag_name} -> [{x}, {y}, {z}]")
        else:
            print(f"AVISO: Formato inválido para {tag_name}. Esperava chaves x, y, z.")

    # Adiciona o RViz (Opcional: Se tiver um arquivo .rviz separado, mude o nome abaixo)
    # Se não tiver arquivo .rviz salvo, ele abrirá o padrão vazio.
    rviz_config_file = os.path.join(script_dir, 'config_visual.rviz') # Nome sugerido para não misturar
    
    # Se o arquivo não existir, abre sem config
    rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen'
    )
    nodes_to_start.append(rviz_node)

    return LaunchDescription(nodes_to_start)

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
