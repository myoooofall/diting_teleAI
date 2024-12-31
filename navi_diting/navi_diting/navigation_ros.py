#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
import threading

class DestinationNavigator(Node):
    def __init__(self):
        super().__init__('navigate_start')

        self.destination_sub = self.create_subscription(
            Int32,
            'destination',
            self.destination_callback,
            10
        )
        self.status=False
        self.current_process = None
        self.current_destination = None
        yolo_command=['python', '/home/diting/liang/ros2_ws/src/navi_diting/navi_diting/Yolov5/run_yolo.py']
        self.yolo_process = subprocess.Popen(yolo_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,text=True)
        
        self.timer=self.create_timer(10.0,self.check_process)
        self.get_logger().info("目标导航节点已启动")

    def destination_callback(self, msg):
        destination = msg.data
        self.get_logger().info(f"收到目标地点: {destination}")
        if self.current_process:
            if destination==0:
                self.get_logger().info("正在终止当前子进程")
                self.current_process.kill()
                self.current_process.wait()
                self.get_logger().info("当前子进程已终止")
                self.current_destination=None
                self.status=False
                self.current_destination = destination
                return
            
            elif destination==self.current_destination:
                self.current_destination = destination
                self.get_logger().info("仍然在导航")
                return
            else:
                self.get_logger().info("正在终止当前子进程")
                self.current_process.kill()
                self.current_process.wait()
                self.get_logger().info("当前子进程已终止")
                command = ['python', '/home/diting/liang/ros2_ws/src/navi_diting/navi_diting/Yolov5/final.py', '--destination', str(destination)]
                self.get_logger().info(f"执行命令: {' '.join(command)}")
                self.current_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,text=True)
                self.current_destination = destination
                self.get_logger().info("打开新导航线程")

        elif destination==0:
            self.current_destination = destination
            return
        else:
            self.get_logger().info(f"导航去{destination}的地方")
            command = ['python', '/home/diting/liang/ros2_ws/src/navi_diting/navi_diting/Yolov5/final.py', '--destination', str(destination)]
            self.get_logger().info(f"执行命令: {' '.join(command)}")
            self.current_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.current_destination = destination
            
    # def handle_process_output(process,logger): 
    #     while True: 
    #         output = process.stdout.readline() 
    #         if output == '' and process.poll() is not None: 
    #             break 
    #         if output: 
    #             self.get_logger().info(output.strip()) 
    #             rc = process.poll() 
    #             return

    def check_process(self):
        
        if self.current_process:
            self.status=self.current_process.poll()
            if self.status is not None:
                self.get_logger().warn("导航成功")
                self.current_process.wait()
                self.current_process = None

def main(args=None):
    rclpy.init(args=args)
    navigator = DestinationNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("节点已关闭")
    finally:
        if navigator.current_process:
            navigator.current_process.terminate()
            navigator.current_process.wait()
        navigator.yolo_process.terminate()
        navigator.yolo_process.wait()
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
