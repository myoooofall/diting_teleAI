#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websocket
import datetime
import hashlib
import base64
import hmac
import json
from urllib.parse import urlencode
import time
import ssl
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread
import os
import wave
import pyaudio
from .voice import synthesize_and_play_audio

class Ws_Param(object):
    # 初始化
    def __init__(self, APPID, APIKey, APISecret, Text):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.Text = Text

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "aisjiuxu", "tte": "utf8"}
        self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-8')), "UTF8")}

    def create_url(self):
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/tts " + "HTTP/1.1"
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (
            self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        url = url + '?' + urlencode(v)
        return url

class XfyunTTSNode(Node):
    def __init__(self):
        super().__init__('xfyun_tts_node')
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            String,
            'gen_voice',
            self.text_callback,
            2
        )
        
        # API参数
        self.APPID = ''
        self.APISecret = ''
        self.APIKey = ''
        
        self.get_logger().info('teleai语音合成节点已启动')

    def text_callback(self, msg):
        """处理接收到的文本消息"""
        text = msg.data
        self.get_logger().info(f'收到文本: {text}')
        synthesize_and_play_audio(text)
        #self.generate_speech(text)

    def generate_speech(self, text):
        """生成语音"""
        wsParam = Ws_Param(
            APPID=self.APPID,
            APISecret=self.APISecret,
            APIKey=self.APIKey,
            Text=text
        )
    

        def on_message(ws, message):
            try:
                message = json.loads(message)
                code = message["code"]
                sid = message["sid"]
                audio = message["data"]["audio"]
                audio = base64.b64decode(audio)
                status = message["data"]["status"]
                
                if code != 0:
                    self.get_logger().error(f"合成错误: {message['message']}")
                    return
                    
                with open('./demo.pcm', 'ab') as f:
                    f.write(audio)
                    
                if status == 2:
                    ws.close()
                    
            except Exception as e:
                self.get_logger().error(f"处理消息出错: {str(e)}")

        def on_error(ws, error):
            self.get_logger().error(f"WebSocket错误: {str(error)}")

        def on_close(ws, close_status_code, close_msg):
            self.get_logger().info("语音合成完成")
            pcm_file = './demo.pcm'
            wav_file = './demo.wav'
            if os.path.exists(pcm_file):
                try:
                    self.get_logger().info("正在转换PCM到WAV...")
                    self.pcm_to_wav(pcm_file, wav_file)
                    self.get_logger().info("开始播放...")
                    self.play_wav(wav_file)
                    self.get_logger().info("播放完成")
                except Exception as e:
                    self.get_logger().error(f"处理音频出错: {str(e)}")

        def on_open(ws):
            def run(*args):
                d = {"common": wsParam.CommonArgs,
                     "business": wsParam.BusinessArgs,
                     "data": wsParam.Data,
                     }
                d = json.dumps(d)
                self.get_logger().info("开始发送文本数据")
                ws.send(d)
                if os.path.exists('./demo.pcm'):
                    os.remove('./demo.pcm')

            thread.start_new_thread(run, ())

        websocket.enableTrace(False)
        wsUrl = wsParam.create_url()
        ws = websocket.WebSocketApp(wsUrl, 
                                  on_message=on_message,
                                  on_error=on_error, 
                                  on_close=on_close)
        ws.on_open = on_open
        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    def pcm_to_wav(self, pcm_file, wav_file, channels=1, sample_width=2, sample_rate=16000):
        """PCM转WAV"""
        with open(pcm_file, 'rb') as pcm_f:
            pcm_data = pcm_f.read()
        with wave.open(wav_file, 'wb') as wav_f:
            wav_f.setnchannels(channels)
            wav_f.setsampwidth(sample_width)
            wav_f.setframerate(sample_rate)
            wav_f.writeframes(pcm_data)

    def play_wav(self, wav_file):
        """播放WAV文件"""
        chunk = 1024
        wf = wave.open(wav_file, 'rb')
        p = pyaudio.PyAudio()
        
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                       channels=wf.getnchannels(),
                       rate=wf.getframerate(),
                       output=True)
        
        data = wf.readframes(chunk)
        while data:
            stream.write(data)
            data = wf.readframes(chunk)
        
        stream.stop_stream()
        stream.close()
        p.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = XfyunTTSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被终止')
if __name__ == '__main__':
    main()
