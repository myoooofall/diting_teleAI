#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
import numpy as np
import sounddevice as sd
import wave
import time
#from scipy.io import wavfile
#from scipy.signal import resample
# import librosa
# import soundfile as sf
from pydub import AudioSegment
print(22)

class AudioDetectorNode(Node):
    def __init__(self):
        super().__init__('audio_detector_node')
        
        # 创建发布者
        self.flag_publisher = self.create_publisher(
            String, 
            'audio_flag', 
            10
        )
        
        # 创建订阅者
        self.done_subscriber = self.create_subscription(
            String,
            'recognition_done',
            self.done_callback,
            10
        )
        
        # 音频参数
        self.sample_rate = 44100
        self.threshold = 0.04
        self.silence_limit = 1
        self.recording = False
        self.recorded_frames = []
        self.last_sound_time = 0
        self.usb_mic_index=self.find_usb_mic()
        # 状态控制
        self.waiting_for_response = False
        
        self.get_logger().info('音频检测节点已启动')
        
    def done_callback(self, msg):
        if msg.data == "DONE":
            self.get_logger().info('收到识别完成信号')
            self.waiting_for_response = False
            
    def audio_callback(self, indata, frames, time_info, status):
        if self.waiting_for_response:
            return
        if status:
            self.get_logger().warn(f"音频状态: {status}")
            
        # 计算音量
        float_data = indata.astype(np.float32) / 32768.0
        volume = np.sqrt(np.mean(float_data ** 2))
        
        if volume > self.threshold:
            if not self.recording:
                self.get_logger().info("检测到声音，开始录音...")
                self.recording = True
                self.recorded_frames = []
            self.recorded_frames.append(indata.copy())
            self.last_sound_time = time.time()
        else:
            if self.recording:
                silence_time = time.time() - self.last_sound_time
                if silence_time > self.silence_limit:
                    self.get_logger().info("停止录音，发送数据...")
                    self.save_and_publish_recording()
                    self.recording = False
                    self.recorded_frames = []
                else:
                    self.recorded_frames.append(indata.copy())
                    
    def save_and_publish_recording(self):

        # current_directory = os.getcwd() # 设置文件路径为绝对路径 
        # filename = os.path.join(current_directory, "temp_audio.wav")
        filename = "/home/diting/liang/ros2_ws/build/temp_audio.wav"
        audio_data = np.concatenate(self.recorded_frames)
        self.get_logger().info(os.path.abspath(filename))
        # 保存音频文件
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_data.tobytes())
        # sr, data = wavfile.read(filename)

        # # 计算目标采样点数量
        # target_sr = 16000
        # num_samples = int(len(data) * target_sr / sr)

        # # 重新采样
        # data_resampled = resample(data, num_samples)

        # # 保存为 16 kHz 的 WAV 文件
        # wavfile.write(filename, target_sr, data_resampled.astype(np.int16))
        # y,sr =librosa.load(filename,sr=44100)
        # y_resampled=librosa.resample(y,orig_sr=sr,target_sr=16000)
        # sf.write(filename,y_resampled,16000)
        audio=AudioSegment.from_file(filename)
        audio =audio.set_frame_rate(16000)
        audio.export(filename,format="wav")

        # 发送开始识别的标志
        flag_msg = String()
        flag_msg.data = "1"
        self.flag_publisher.publish(flag_msg)
        # 获取当前时间并格式化
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.get_logger().info(f"[{current_time}] 音频数据已发送")
        self.waiting_for_response = True
        
    def start_monitoring(self):
        
        with sd.InputStream(callback=self.audio_callback,
                          channels=1,
                          dtype='int16',
                          samplerate=self.sample_rate,
                          device=self.usb_mic_index):
            try:
                while rclpy.ok():
                    rclpy.spin_once(self)
            except KeyboardInterrupt:
                self.get_logger().info("停止监听")
                    
    def find_usb_mic(self):
        devices = sd.query_devices()
        print("\n可用的音频设备:")
        print("-" * 60)
        
        usb_mic_index = None
        
        for i, dev in enumerate(devices):
            # 只显示输入设备
            if dev['max_input_channels'] > 0:
                print(f"设备 {i}: {dev['name']}")
                print(f"   ALSA设备: {dev.get('hostapi_name')} - {dev.get('name')}")
                print(f"   输入通道: {dev['max_input_channels']}")
                print(f"   采样率: {dev['default_samplerate']}Hz")
                print("-" * 60)
                
                if 'usb' in dev['name'].lower() and 'mic' in dev['name'].lower():
                    usb_mic_index = i
        
        if usb_mic_index is not None:
            print(f"\n找到USB麦克风! sounddevice设备号: {usb_mic_index}")
        else:
            print("\n未找到USB麦克风设备")
        
        return usb_mic_index
                
def main(args=None):
    rclpy.init(args=args)
    detector = AudioDetectorNode()
    
    try:
        detector.start_monitoring()
    except Exception as e:
        detector.get_logger().error(f"错误: {e}")
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
