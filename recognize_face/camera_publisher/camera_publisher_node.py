# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image
# # from std_msgs.msg import String
# # from sensor_msgs.msg import Image
# # from cv_bridge import CvBridge
# # import numpy as np
# # import pyrealsense2 as rs
# # import cv2
# # #import torch
# # from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# # qos_profile = QoSProfile(
# #     reliability=ReliabilityPolicy.RELIABLE,  # 设置为可靠传输
# #     history=HistoryPolicy.KEEP_LAST,  # 保持最后的消息
# #     depth=20  # 缓冲区深度
# # )

# # class CameraPublisherSubscriberNode(Node):
# #     def __init__(self):
# #         super().__init__('camera_publisher_subscriber_node')
        
# #         # 创建一个发布者，发布图像消息
# #         self.publisher_ = self.create_publisher(Image, 'camera/image_raw', qos_profile)
        
# #         # 创建一个订阅者，订阅中文消息
# #         self.subscription = self.create_subscription(
# #             String,
# #             'image_processed_info',
# #             self.string_callback,
# #             qos_profile)

# #         # 创建一个定时器，每秒发布一次图像
# #         self.timer = self.create_timer(1.0, self.timer_callback)
        
# #         # 创建CV桥接器
# #         self.bridge = CvBridge()

# #         # 设置RealSense管道
# #         self.pipeline = rs.pipeline()
# #         config = rs.config()
# #         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# #         self.pipeline.start(config)

# #     def timer_callback(self):
# #         # 获取图像帧
# #         frames = self.pipeline.wait_for_frames()
# #         color_frame = frames.get_color_frame()

# #         # 将RealSense图像数据转换为OpenCV格式
# #         color_image = np.asanyarray(color_frame.get_data())

# #         # 使用cv_bridge将OpenCV图像转换为ROS图像消息
# #         try:
# #             msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
# #             self.publisher_.publish(msg)
# #             self.get_logger().info('Publishing image...')
# #         except Exception as e:
# #             self.get_logger().error(f"Error converting image: {e}")

# #     def string_callback(self, msg):
# #         # 处理接收到的中文消息
# #         try:
# #             self.get_logger().info(f"Received message: {msg.data}")
# #         except Exception as e:
# #             self.get_logger().error(f"Error processing string message: {e}")

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = CameraPublisherSubscriberNode()
# #     rclpy.spin(node)
# #     node.pipeline.stop()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
# import rclpy
# from rclpy.node import Node
# from cv_bridge import CvBridge
# import numpy as np
# import pyrealsense2 as rs
# import cv2
# import socket

# class RealSenseUDPSenderNode(Node):
#     def __init__(self):
#         super().__init__('realsense_udp_sender_node')

#         # 创建定时器，每秒发送一张图像
#         self.timer = self.create_timer(1.0, self.timer_callback)  # 1.0秒定时器

#         # CVBridge 用于转换图像格式
#         self.bridge = CvBridge()

#         # 设置 RealSense 管道
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         self.pipeline.start(config)

#         # 设置 UDP 套接字
#         self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.target_ip = '192.168.155.45'  # 替换为接收端的 IP 地址
#         self.target_port = 12345  # 接收端的端口号

#     def timer_callback(self):
#         try:
#             # 获取 RealSense 图像帧
#             frames = self.pipeline.wait_for_frames()
#             color_frame = frames.get_color_frame()

#             if not color_frame:
#                 self.get_logger().warn('No frame received from RealSense!')
#                 return

#             # 将 RealSense 图像转换为 NumPy 格式
#             color_image = np.asanyarray(color_frame.get_data())

#             # 使用 OpenCV 压缩图像为 JPEG 格式
#             _, encoded_image = cv2.imencode('.jpg', color_image)

#             # 通过 UDP 发送图像数据
#             self.udp_socket.sendto(encoded_image.tobytes(), (self.target_ip, self.target_port))
#             self.get_logger().info('Sent one image via UDP.')
#         except Exception as e:
#             self.get_logger().error(f"Error in timer_callback: {e}")

#     def destroy_node(self):
#         # 停止 RealSense 管道
#         self.pipeline.stop()
#         # 关闭 UDP 套接字
#         self.udp_socket.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = RealSenseUDPSenderNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()





import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs
import cv2
import socket
import threading
from std_msgs.msg import String, ByteMultiArray


class RealSenseUDPSenderNode(Node):
    def __init__(self):
        super().__init__('realsense_udp_sender_node')

        # 标志变量，用于控制是否可以发送下一张图像
        self.can_send_image = True  # 初始设置为True，允许发送第一次图像

        # 创建定时器，每秒检查是否可以发送图像
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1.0秒定时器

        # CVBridge 用于转换图像格式
        self.bridge = CvBridge()

        # 设置 RealSense 管道
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 设置 UDP 套接字
        self.udp_socket_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_ip = '192.168.155.45'  # 替换为接收端的 IP 地址
        self.target_port_send = 12345  # 接收端的端口号，用于发送图像

        # 设置接收 UDP 套接字
        self.udp_socket_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_port_recv = 12346  # 接收端的端口号，用于接收字符串
        self.udp_socket_recv.bind(('192.168.161.185', self.target_port_recv))  # 绑定本机端口来接收数据

        # 创建一个线程用于接收 UDP 数据
        self.recv_thread = threading.Thread(target=self.receive_udp_data, daemon=True)
        self.recv_thread.start()

        self.name_publisher = self.create_publisher(
            String,
            'name',
            10
        )

    def timer_callback(self):
        """ 检查是否可以发送下一张图像 """
        if self.can_send_image:
            try:
                # 获取 RealSense 图像帧
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                if not color_frame:
                    self.get_logger().warn('No frame received from RealSense!')
                    return

                # 将 RealSense 图像转换为 NumPy 格式
                color_image = np.asanyarray(color_frame.get_data())

                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # 设置压缩质量为50，范围 0-100
                _, encoded_image = cv2.imencode('.jpg', color_image, encode_param)

                # 使用 OpenCV 压缩图像为 JPEG 格式
                #_, encoded_image = cv2.imencode('.jpg', color_image)

                # 通过 UDP 发送图像数据
                self.udp_socket_send.sendto(encoded_image.tobytes(), (self.target_ip, self.target_port_send))
                self.get_logger().info('Sent one image via UDP.')

                # 发送完图像后将标志设为 False，等待下一次接收字符串
                self.can_send_image = False
            except Exception as e:
                self.get_logger().error(f"Error in timer_callback: {e}")

    def receive_udp_data(self):
        """ 接收来自目标端的字符串并保存到文件 """
        while rclpy.ok():
            try:
                # 接收数据
                data, addr = self.udp_socket_recv.recvfrom(1024)  # 最大接收1024字节
                received_message = data.decode('utf-8')
                self.get_logger().info(f"Received message: {received_message}")
                name_msg = String()
                name_msg.data = received_message
                self.name_publisher.publish(name_msg)
                # 将接收到的字符串写入文件（覆盖模式）
                with open('received_data.txt', 'w', encoding='utf-8') as file:
                    file.write(received_message)
                

                # 收到字符串后，将标志变量设置为 True，允许发送下一张图像
                self.can_send_image = True
            except Exception as e:
                self.get_logger().error(f"Error in receive_udp_data: {e}")

    def destroy_node(self):
        # 停止 RealSense 管道
        self.pipeline.stop()
        # 关闭发送和接收的 UDP 套接字
        self.udp_socket_send.close()
        self.udp_socket_recv.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseUDPSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

