import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Obter o caminho para os arquivos de configuração do seu pacote
    pkg_name = 'limo_apriltag_tools'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # Caminho do arquivo de parâmetros AprilTag
    apriltag_params_file = os.path.join(
        pkg_share_dir,
        'config',
        'apriltag_params.yaml'
    )
    
    ### ALTERAÇÃO: Encontrar o caminho para o seu arquivo de calibração ###
    # (Assumindo que você moveu 'ost.txt' para 'config/webcam_calibration.yaml')
    calibration_file = os.path.join(
        pkg_share_dir,
        'config',
        'webcam_calibration.yaml'
    )
    
    # --- 1. NÓ DA CÂMERA (v4l2_camera_node) ---
    # Objetivo: Publicar a imagem (/image_raw) e desativar o tópico /camera_info padrão.
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        remappings=[
            # A câmera publica o image_raw, mas remapamos o camera_info inválido
            ('/camera_info', '/camera_info_junk'), 
        ],
        parameters=[
            # Aqui você pode adicionar parâmetros específicos da câmera, se houver
        ]
    )

    # --- 2. NÓ DA CALIBRAÇÃO (Seu Script Sincronizado) ---
    # Objetivo: Publicar o CameraInfo correto no tópico /camera_info
    calibration_publisher_node = Node(
        package=pkg_name,
        executable='camera_info_publisher_node', # Nome do executável Python no setup.py
        name='camera_info_publisher',
        output='screen',
        ### ALTERAÇÃO: Passar o caminho do arquivo de calibração como um parâmetro ROS ###
        parameters=[
            {'calibration_file': calibration_file}
        ]
    )

    # --- 3. NÓ APRILTAG ---
    # Objetivo: Processar a imagem e a calibração, usando sincronização aproximada.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        # Remapeamento para usar os tópicos corretos
        remappings=[
            ('image_rect', '/image_raw'),      # O AprilTag consome a imagem bruta
            ('camera_info', '/camera_info'),   # Consome o CameraInfo do seu script
        ],
        # Carregar o arquivo YAML e forçar os parâmetros de sincronização
        parameters=[
            apriltag_params_file,
            {'sync_approximate': True},             # Usar sincronização aproximada
            {'approximate_sync_slop': 0.5},         # Tolerância de 0.5 segundos
        ]
    )

    # Retornar a descrição de lançamento com todos os nós
    return LaunchDescription([
        camera_node,
        calibration_publisher_node,
        apriltag_node,
    ])
