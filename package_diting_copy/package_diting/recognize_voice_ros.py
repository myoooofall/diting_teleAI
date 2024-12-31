#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
import wave

class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__('voice_recognition_node')
        
        self.flag_subscriber = self.create_subscription(
            String,
            'audio_flag',
            self.flag_callback,
            10
        )
        
        # 创建发布者
        self.done_publisher = self.create_publisher(
            String,
            'recognition_done',
            10
        )
        self.instruction_publisher = self.create_publisher(
            String,
            'instruction',
            10
        )
        self.get_logger().info('语音识别节点已启动')
        
        
    def flag_callback(self, msg):
        if msg.data == "1":
            self.process_audio()
    
    def recognize_voice(self,filename):
        print("识别中...")
        #识别完发给语言模型 接收最后消息 然后发声 然后publish指示
        return 
            
    def process_audio(self):
        try:
            # 保存接收到的音频
            filename = "temp_audio.wav"
            self.recognize_voice(filename)
            
            # 发送完成信号
            done_msg = String()
            done_msg.data = "DONE"
            self.done_publisher.publish(done_msg)
            self.get_logger().info("done")
            
        except Exception as e:
            self.get_logger().error(f"识别错误: {e}")
    

def main(args=None):
    rclpy.init(args=args)
    recognizer = VoiceRecognitionNode()
    
    try:
        rclpy.spin(recognizer)
    except KeyboardInterrupt:
        pass
    finally:
        recognizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
