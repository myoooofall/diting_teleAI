#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading
import time

class TestNavigator(Node):
    def __init__(self):
        super().__init__('test_navigator')
        self.publisher = self.create_publisher(Int32, 'destination', 10)
        self.get_logger().info("测试导航节点已启动")
        self.running = True

    def publish_destination(self, destination):
        msg = Int32()
        msg.data = destination
        self.publisher.publish(msg)
        self.get_logger().info(f"发布目标地点: {destination}")

    def user_input_thread(self):
        while self.running:
            try:
                destination = int(input("请输入目标地点 (0: 关闭, 1: 目标1, 2: 目标2): "))
                self.publish_destination(destination)
                if destination == -1:  # -1 用于退出测试
                    self.running = False
            except ValueError:
                print("请输入有效的整数")

def main(args=None):
    rclpy.init(args=args)
    test_navigator = TestNavigator()

    try:
        input_thread = threading.Thread(target=test_navigator.user_input_thread)
        input_thread.start()
        rclpy.spin(test_navigator)
    except KeyboardInterrupt:
        test_navigator.get_logger().info("测试节点已关闭")
    finally:
        test_navigator.running = False
        input_thread.join()
        test_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()