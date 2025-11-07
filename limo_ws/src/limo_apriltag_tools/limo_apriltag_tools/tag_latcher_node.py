#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

# Mensagens de Geometria e TF2
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# --- Configurações Padrão ---
# --- CORREÇÃO AQUI ---
FRAME_FIXO_PADRAO = "map"     # Removida a barra inicial (era "/map")
# --- FIM DA CORREÇÃO ---

PREFIXO_TAG_PADRAO = "tag36h11:" # O que você definiu no launch file
MAX_TAG_ID_PADRAO = 50         # Quantos IDs de tag vamos "policiar"

class TagLatcherNode(Node):
    def __init__(self):
        super().__init__('tag_latcher_node')
        self.get_logger().info("Iniciando o nó Latcher de TF (Modo Polling)...")

        # --- Parâmetros ---
        self.fixed_frame = self.declare_parameter(
            'fixed_frame', FRAME_FIXO_PADRAO).get_parameter_value().string_value
        self.tag_prefix = self.declare_parameter(
            'tag_prefix', PREFIXO_TAG_PADRAO).get_parameter_value().string_value
        self.max_tag_id = self.declare_parameter(
            'max_tag_id', MAX_TAG_ID_PADRAO).get_parameter_value().integer_value
        
        self.last_seen_poses = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.tf_update_and_publish_callback)

        self.get_logger().info(f"Nó Latcher configurado.")
        self.get_logger().info(f"  -> Frame Fixo (Mundo): {self.fixed_frame}")
        self.get_logger().info(f"  -> Policiando Tags: {self.tag_prefix}[0...{self.max_tag_id}]")


    def tf_update_and_publish_callback(self):
        """
        Callback acionado pelo Timer (10Hz).
        1. Tenta ATUALIZAR a pose de todas as tags visíveis.
        2. RE-PUBLICA todas as poses salvas na memória.
        """
        now = self.get_clock().now()
        
        for i in range(self.max_tag_id + 1):
            tag_frame = self.tag_prefix + str(i)
            
            try:
                # Pergunta ao TF2: "Qual a transformação completa do map -> target_i?"
                t = self.tf_buffer.lookup_transform(
                    self.fixed_frame,       # Frame Alvo (Pai) - agora "map"
                    tag_frame,             # Frame Fonte (Filho)
                    Time(),                # Pega o mais recente
                    timeout=Duration(seconds=0.05)
                )
                
                self.last_seen_poses[tag_frame] = t
                
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                pass # Erro é normal, significa que a tag não está visível

        # --- 2. RE-PUBLICAR Poses da Memória ---
        if not self.last_seen_poses:
            return

        now_msg = now.to_msg()

        for tag_frame, stored_tf in self.last_seen_poses.items():
            t_out = stored_tf
            t_out.header.stamp = now_msg
            self.tf_broadcaster.sendTransform(t_out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TagLatcherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
