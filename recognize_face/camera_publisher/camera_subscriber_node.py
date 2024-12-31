import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # 设置为可靠传输
    history=HistoryPolicy.KEEP_LAST,  # 保持最后的消息
    depth=20  # 缓冲区深度
)

class CameraSubscriberPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber_publisher_node')

        # 创建图像消息订阅者
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos_profile)
        
        # 创建中文字符串发布者
        self.string_publisher = self.create_publisher(String, 'image_processed_info', qos_profile)

        # 初始化CvBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 显示图像
            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(1)  # 1毫秒的等待时间，确保图像窗口更新

            # 发布中文字符串
            string_msg = String()
            string_msg.data = "刘洋"  # 中文消息
            self.string_publisher.publish(string_msg)
            self.get_logger().info('发布人名: "刘洋"')

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberPublisherNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()  # 关闭OpenCV图像窗口
    rclpy.shutdown()

if __name__ == '__main__':
    main()
