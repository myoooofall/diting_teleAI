#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from final import navigation  # 导入navigation函数

class DestinationNavigator(Node):
    def __init__(self):
        # 初始化ROS2节点
        super().__init__('destination_navigator')
        
        # 创建订阅者，订阅destination话题
        self.destination_sub = self.create_subscription(
            Int32,
            'destination',
            self.destination_callback,
            10  # QoS配置
        )
        
        self.get_logger().info("目标导航节点已启动")

    def destination_callback(self, msg):
        """
        处理接收到的目标点消息
        """
        destination = msg.data
        self.get_logger().info(f"收到新的目标点: {destination}")
        navigation(destination)
        self.get_logger().info(f"导航到目标点 {destination} 完成")
        

def main(args=None):
    rclpy.init(args=args)
    
    navigator = DestinationNavigator()
    
    try:
        # 保持节点运行
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("节点已关闭")
    finally:
        # 清理节点
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()