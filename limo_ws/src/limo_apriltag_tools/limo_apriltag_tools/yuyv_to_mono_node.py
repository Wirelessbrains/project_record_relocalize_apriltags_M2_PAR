import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import copy
import traceback
import time

class YuyvToMono(Node):
    def __init__(self):
        super().__init__('yuyv_to_mono')

        # QoS confiável (para garantir que /camera_info chegue)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_img  = self.create_subscription(Image, '/image_raw', self.img_cb, qos_reliable)
        self.sub_info = self.create_subscription(CameraInfo, '/camera_info', self.info_cb, qos_reliable)
        self.pub_img  = self.create_publisher(Image, '/image_mono', qos_reliable)
        self.pub_info = self.create_publisher(CameraInfo, '/camera_info_mono', qos_reliable)

        self._last_info = None
        self._logged_encoding = False
        self._first_info_warned = False
        self._start_time = time.time()

    def info_cb(self, msg: CameraInfo):
        self._last_info = msg

    def _synthesize_info(self, img_msg: Image) -> CameraInfo:
        info = CameraInfo()
        info.header = img_msg.header
        info.height = img_msg.height
        info.width  = img_msg.width
        info.k = [1.0, 0.0, info.width/2.0,
                  0.0, 1.0, info.height/2.0,
                  0.0, 0.0, 1.0]
        info.p = [1.0, 0.0, info.width/2.0, 0.0,
                  0.0, 1.0, info.height/2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.d = []
        info.distortion_model = 'plumb_bob'
        info.r = [1.0,0.0,0.0,
                  0.0,1.0,0.0,
                  0.0,0.0,1.0]
        return info

    def img_cb(self, msg: Image):
        try:
            w, h = msg.width, msg.height
            enc = (msg.encoding or '').lower()

            if not self._logged_encoding:
                self.get_logger().info(f"Entrada /image_raw encoding='{enc}'")
                self._logged_encoding = True

            data = np.frombuffer(msg.data, dtype=np.uint8)

            # ---------- Converter para mono8 ----------
            if enc in ('yuyv', 'yuy2', 'yuv422', 'yuv422_yuy2'):
                y = data[0::2].reshape((h, w))
            elif enc in ('uyvy', 'yuv422_uyvy'):
                y = data[1::2].reshape((h, w))
            elif enc in ('rgb8', 'bgr8'):
                img3 = data.reshape((h, w, 3)).astype(np.uint16)
                if enc == 'rgb8':
                    R, G, B = img3[:,:,0], img3[:,:,1], img3[:,:,2]
                else:
                    B, G, R = img3[:,:,0], img3[:,:,1], img3[:,:,2]
                y = ((77*R + 150*G + 29*B) >> 8).astype(np.uint8)
            else:
                if data.size == w*h*2:
                    y = data[0::2].reshape((h, w))
                    if not self._logged_encoding:
                        self.get_logger().warn(f"Encoding '{enc}' desconhecido; usando fallback Y=pares.")
                else:
                    self.get_logger().error(f"Encoding '{enc}' não suportado (size {data.size}).")
                    return

            # ---------- Publica imagem mono ----------
            out = Image()
            out.header = msg.header
            out.height = h
            out.width = w
            out.encoding = 'mono8'
            out.is_bigendian = 0
            out.step = w
            out.data = y.tobytes()
            out.header.frame_id = 'camera'  # ⚠️ força frame_id consistente
            self.pub_img.publish(out)

            # ---------- CameraInfo sincronizado ----------
            if self._last_info is None:
                if (not self._first_info_warned) and (time.time() - self._start_time > 1.0):
                    self.get_logger().warn("Nenhum /camera_info recebido ainda; publicando CameraInfo sintético até o real chegar.")
                    self._first_info_warned = True
                info = self._synthesize_info(msg)
            else:
                info = copy.copy(self._last_info)
                info.height = h
                info.width  = w

            info.header.stamp = msg.header.stamp
            info.header.frame_id = 'camera'  # ⚠️ força frame_id consistente
            self.pub_info.publish(info)

        except Exception as e:
            self.get_logger().error(f"Exceção no img_cb: {e}\n{traceback.format_exc()}")

def main():
    rclpy.init()
    node = YuyvToMono()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

