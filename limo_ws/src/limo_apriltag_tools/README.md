limo_apriltag_tools
====================
AprilTag Detection & Camera Calibration Module  
Part of the Wireless Brains – LIMO Autonomy Project (M2 PAR)

---------------------------------------------------------------------
Overview
---------------------------------------------------------------------
This package provides AprilTag detection and camera calibration synchronization 
for the AgileX LIMO robot. It serves as the visual perception layer of the system — 
detecting fiducial markers in real time and publishing both the tag poses and 
camera parameters to other modules (mapping, localization, navigation).

---------------------------------------------------------------------
Main Features
---------------------------------------------------------------------
Component | Description
-----------|---------------------------------------------------------
v4l2_camera_node | Captures images from the onboard webcam and publishes /image_raw
camera_info_publisher_node | Publishes /camera_info messages synchronized with image timestamps using calibration data from YAML
apriltag_node (apriltag_ros) | Detects AprilTags and publishes their poses on /detections and TF frames (km_0, km_1, ...)

---------------------------------------------------------------------
Launch Command
---------------------------------------------------------------------
ros2 launch limo_apriltag_tools apriltag_webcam_full.launch.py

This launches:
- v4l2_camera_node – image capture  
- camera_info_publisher_node – synchronized calibration  
- apriltag_node – tag detection & TF publishing

---------------------------------------------------------------------
Configuration Files
---------------------------------------------------------------------
File | Purpose
------|--------------------------------------------------------------
config/apriltag_params.yaml | Defines tag family, size, and frame IDs (km_0, km_1, …)
config/webcam_calibration.yaml | Intrinsic calibration (K, D, R, P) from the camera

Example AprilTag config:
---------------------------------------------------------------
apriltag_node:
  ros__parameters:
    # --- TÓPICOS DE ENTRADA ---
    # Os tópicos onde o nó espera receber a imagem e as informações da câmera.
    image_topic: '/camera/image_rect'          # Tópico da imagem retificada.
    camera_info_topic: '/camera/camera_info'   # Tópico das informações da câmera.

    # --- SINC. & IMAGEM ---
    # Parâmetros de sincronização
    sync_approximate: True
    # Se 'False' (padrão), usa sincronização exata, que é mais rigorosa.
    # 'True' (sincronização aproximada) é geralmente necessário para câmeras USB/Webcam.
    
    # Este parâmetro garante que a tolerância seja aplicada corretamente:
    use_image_transport: True
    # Usa image_transport para lidar com diferentes formatos de imagem (ex: raw, compressed).

    # --- TAGS DE ENTRADA ---
    # Família de tags a ser detectada.
    tag_family: 'tag36h11'
    # Opções comuns: 'tag36h11', 'tag25h9', 'tag16h5', etc.
    
    # Tamanho da tag: **CRUCIAL**
    tag_size: 0.025 # Use o tamanho da **ARESTA DO QUADRADO PRETO** em METROS! (0.025m = 2.5cm)
    
    # IDs das tags a serem ignoradas (opcional)
    # ignore_ids: [0, 5, 10]
    
    # --- FRAMES DE SAÍDA ---
    # O frame de referência (pai) para a transformação publicada (TF).
    # Geralmente é o frame óptico da câmera.
    camera_frame: 'camera_link_optical'
    
    # O prefixo do frame para as tags detectadas. 
    # O frame final será: <tag_frame_prefix><tag_id> (ex: 'tag_36h11_0')
    tag_frame_prefix: 'tag_' 
---------------------------------------------------------------------
Visualization
---------------------------------------------------------------------
Launch RViz and add:
- Image display → topic /image_raw
- MarkerArray display → topic /detections

---------------------------------------------------------------------
Purpose in the LIMO Autonomy Project
---------------------------------------------------------------------
This module is used by:
- limo_mapping to record AprilTag positions during teleoperation (Teach phase);
- limo_relocalization to re-detect and reorient the robot relative to known AprilTags (Resume phase);
- limo_route_follow to localize and move toward checkpoints based on tags (Replay phase).

---------------------------------------------------------------------
License
---------------------------------------------------------------------
MIT License © Wireless Brains 2025  
Part of the LIMO Autonomy Project (M2 PAR), Université Clermont Auvergne.

