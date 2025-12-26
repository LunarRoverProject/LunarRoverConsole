import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# QoS関連のインポートを追加
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ImageViewer(Node):
    """
    /camera/image_rawトピックをサブスクライブして映像を表示するノード
    """
    def __init__(self):
        super().__init__('image_viewer_node')
        
        # 1. 画像ストリームに適したQoSプロファイルを作成
        image_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # ここを BEST_EFFORT に設定
            history=1,  # 最新のフレームのみ保持 (KEEP_LAST)
            depth=1
        )
        
        # 2. サブスクライバにQoSプロファイルを指定
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.image_callback,
            qos_profile=image_qos_profile) # qos_profile を追加
            
        self.subscription  
        
        # CvBridgeのインスタンス化
        self.bridge = CvBridge()
        
        self.get_logger().info('Image viewer node has been started with BEST_EFFORT QoS.')

    def image_callback(self, msg):
        # ... (中略：コールバック関数は変更なし) ...
        self.get_logger().info('Receiving video frame')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
            
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_viewer = ImageViewer()
    
    try:
        rclpy.spin(image_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        image_viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()