import os
import json
import base64
import asyncio
import websockets
import ssl
import wave
import logging
from playsound import playsound
import sys

# logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# 对输入文本序列进行切分的最大长度
CHUNK_SIZE = 6400

def print_exception_details(e):
    exc_type, exc_value, exc_traceback = sys.exc_info()
    logging.error("pid: {} info: {}".format(os.getpid(), e))
    logging.error("Error Type: {}".format(exc_type))
    logging.error("Error Value: {}".format(exc_value))
    logging.error("Traceback:")
    while exc_traceback:
        logging.error("  File \"{}\", line {}, in {}".format(exc_traceback.tb_frame.f_code.co_filename, exc_traceback.tb_lineno, exc_traceback.tb_frame.f_code.co_name))
        exc_traceback = exc_traceback.tb_next


async def uws_call_and_play_audio(text, url, X_APP_ID, X_APP_KEY, Order_Num):
    combined_audio_base64 = []  # 用于存储所有接收到的信息

    async def uws_recv(websocket):
        while True:
            try:
                message = await websocket.recv()
                message = json.loads(message)

                if "result" in message and "audio" in message["result"]:
                    combined_audio_base64.append(message["result"]["audio"])

                if "result" in message and message["result"].get("is_end", False):
                    break
            except Exception as e:
                print("Error receiving message:", e)
                break

    ssl_context = ssl._create_unverified_context()
    try:
        async with websockets.connect(url, extra_headers={
            'X-APP-ID': X_APP_ID,
            'X-APP-KEY': X_APP_KEY,
            'Order-Num': Order_Num,
            'Content-Type': 'application/json'
        }, ssl=ssl_context) as websocket:
            start_json = {
                'req_id': "3a87fe9793c9-4ebd-95d4-4ce2-a80c054b",
                'text': text,
                'voice': 'yixiaoling',
                'speech_rate': 1.0,
                'pitch': 1.5,
                'language': 2
            }
            await websocket.send(json.dumps(start_json))
            await uws_recv(websocket)

    except Exception as e:
        logging.error(f"Error: {e}")
        print_exception_details(e)

    # 合并所有音频片段的 Base64 编码
    final_audio_base64 = ''.join(combined_audio_base64)
    output_wav = "./output_audio.wav"

    # 将合并的音频保存为 WAV 文件
    save_audio_as_wav(final_audio_base64, output_wav)
    print(f"Audio saved to {output_wav}")
    playsound(output_wav)


# **保存 Base64 音频为 WAV 文件**
def save_audio_as_wav(base64_audio, output_wav):
    audio_data = base64.b64decode(base64_audio)  # 解码 Base64 数据
    with wave.open(output_wav, 'wb') as wav_file:
        # 假设音频格式为 16kHz, 单声道, 16 位深度
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(16000)
        wav_file.writeframes(audio_data)
    print(f"Audio successfully saved as {output_wav}")


def synthesize_and_play_audio(text, url='',
                              X_APP_ID='', X_APP_KEY='', Order_Num=''):
    asyncio.run(uws_call_and_play_audio(text, url, X_APP_ID, X_APP_KEY, Order_Num))

def main():
    synthesize_and_play_audio("你好泥豪你好")


if __name__ == '__main__':
    main()

