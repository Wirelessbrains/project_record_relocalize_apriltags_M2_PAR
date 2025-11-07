import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image # Adicionamos Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import os

# 🎯 CAMINHO FIXO REMOVIDO! 
# O caminho agora será recebido como um parâmetro ROS.

class SynchronizedCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # --- 1. Declarar e Obter o Parâmetro do Arquivo de Calibração ---
        self.declare_parameter('calibration_file', '') # Declara o parâmetro
        
        # Obtém o caminho do arquivo (definido no launch file)
        calibration_file_path = self.get_parameter('calibration_file').get_parameter_value().string_value

        if not calibration_file_path:
            self.get_logger().error("ERRO: O parâmetro 'calibration_file' não foi definido! Encerrando.")
            return # Para a inicialização se o parâmetro não for passado

        # --- Configuração do QoS ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. Carregar a Mensagem de Calibração (usando o caminho do parâmetro)
        self.camera_info_msg = self._load_camera_info_from_yaml(calibration_file_path)
        
        if self.camera_info_msg is None:
            self.get_logger().error("ERRO: O nó não pode ser iniciado sem dados de calibração válidos.")
            return

        # 3. Configurar o Publicador de CameraInfo
        self.pub_camera_info = self.create_publisher(
            CameraInfo, 
            '/camera_info', 
            qos_profile
        )

        # 4. Configurar o Subscritor da Imagem (Para Forçar a Sincronização)
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self._image_callback, 
            10 # QoS depth
        )

        self.get_logger().info(f"Nó de Calibração Sincronizada iniciado, usando {calibration_file_path}")


    def _load_camera_info_from_yaml(self, file_path): # Recebe o file_path como argumento
        # 
        # --- 🔴 ERRO CORRIGIDO AQUI ---
        # A docstring de três aspas agora está fechada corretamente.
        #
        """Carrega e preenche a mensagem CameraInfo a partir do YAML."""
        
        try:
            # Usa o argumento file_path em vez da variável global
            with open(file_path, 'r') as file:
                calib_data = yaml.safe_load(file)
        except FileNotFoundError:
            # Usa file_path na mensagem de erro
            self.get_logger().error(f"Arquivo de calibração não encontrado: {file_path}")
            return None
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar o YAML: {e}")
            return None

        cinfo = CameraInfo()
        
        cinfo.height = calib_data['image_height']
        cinfo.width = calib_data['image_width']
        cinfo.distortion_model = calib_data['distortion_model']
        cinfo.k = calib_data['camera_matrix']['data']
        cinfo.d = calib_data['distortion_coefficients']['data']
        cinfo.r = calib_data['rectification_matrix']['data']
        cinfo.p = calib_data['projection_matrix']['data']
        
        # Frame de Referência: CORRIGIDO para 'camera'
        cinfo.header.frame_id = 'camera_link_optical' 
        
        self.get_logger().info("Calibração da câmera carregada com sucesso via PyYAML.")
        return cinfo


    def _image_callback(self, img_msg):
        """Callback acionado por novas imagens. Copia o header e publica o CameraInfo."""
        
        if self.camera_info_msg is None:
            return

        # --- SINCRONIZAÇÃO CRUCIAL ---
        self.camera_info_msg.header = img_msg.header
        self.pub_camera_info.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedCameraInfoPublisher() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
