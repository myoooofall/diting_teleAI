#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
import wave
from package_diting.iat_ws_python3 import xfyun_init, xfyun_recognize
from package_diting.call_with_prompt_fin import llm_init,get_reply


class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__('voice_recognition_node')
        self.filename = '/home/diting/liang/ros2_ws/build/temp_audio.wav'
        xfyun_init(self.filename)  # 这里改成你的音频文件路径
        self.chat = llm_init()
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
            3
        )
        self.voice_publisher = self.create_publisher(
            String,
            'gen_voice',
            2
        )
        self.face_subscriber = self.create_subscription(
            String,
            'name',
            self.face_callback,
            2
        )
        self.get_logger().info('语音识别节点已启动')
        
        
    def flag_callback(self, msg):
        if msg.data == "1":
            self.process_audio()
            done_msg = String()
            done_msg.data = "DONE"
            self.done_publisher.publish(done_msg)
    
    def face_callback(self, msg):
        name = msg.data
        action = "打招呼"
        reply = ""
        
        if name[0]=='N':
            return
        reply = name[0] + "总,您好，很高兴见到您"
        
        self.get_logger().info(f"人名: {name}, 回复: {reply}")

        action_msg = String()
        action_msg.data = action
        self.instruction_publisher.publish(action_msg)

        voice_msg = String()
        voice_msg.data = reply
        self.voice_publisher.publish(voice_msg)

    
    def recognize_voice(self):
        print("识别中...")
        result = xfyun_recognize() 
        self.get_logger().info(result)
        action,reply = get_reply(self.chat,result)
        #识别完发给语言模型 接收最后消息 然后发声 然后publish指示
        return action,reply
            
    def process_audio(self):
        try:
            # 保存接收到的音频
            action,reply=self.recognize_voice()
            print(f"action:::{action}")
            #action,reply=self.recognize_voice()
            if reply is not None:
                reply = str(reply)
                voice_msg=String()
                voice_msg.data=reply
                self.voice_publisher.publish(voice_msg)
                self.get_logger().info(f"publish voice: {voice_msg.data}")
            if action is not None:    
                action = str(action) 
                instruction_msg = String()
                instruction_msg.data = action
                self.instruction_publisher.publish(instruction_msg)
                self.get_logger().info(f"发布指令: {instruction_msg.data}")
                return
            else:
                self.get_logger().info(f"未识别到有效指令")
            
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
