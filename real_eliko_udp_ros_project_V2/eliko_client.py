import matlab.engine
import socket
import numpy as np
import signal
import sys
import time
import csv

# 全局变量
SEQ_NUM = 0
TEST = False  # 设置 True 可测试模拟数据 False
DESIRED_HZ = 100  # 发送频率
DELAY = 1.0 / DESIRED_HZ
UDP_IP = "192.168.1.134"  # 服务器端的 IP
UDP_PORT = 1234  # 与 ROS 订阅端口一致

# 信号处理函数 Ctrl+C 停止程序
def signal_handler(sig, frame):
    print("Interrupted. Stopping...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# 初始化 MATLAB 引擎
print("Starting MATLAB engine...")
eng = matlab.engine.start_matlab()
eng.cd(r'D:/github/EBI/real_eliko_udp_ros_project_V2', nargout=0)

# 启动 socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 主函数
def run_ebi_server():
    global SEQ_NUM

    print("EBI UDP server started...")
    with open('code/src/teleop_enhancer_pkg/Log/ebi/ebi_data_server.csv', 'w', newline='') as file:
        writer = csv.writer(file, delimiter=';')
        writer.writerow(['seq', 'timestamp', 'data'])

        while True:
            start_time = time.time()

            if not TEST:
                try:
                    complex_mat = eng.ElikoSample()
                    complex_array = np.array(complex_mat._data).reshape(complex_mat.size, order='F')
                except Exception as e:
                    print("MATLAB error:", e)
                    continue
            else:
                # 模拟复数数据
                complex_array = np.array([1+1j, 2+2j, 3+3j])

            # 构造消息
            real_parts = complex_array.real.tolist()
            imag_parts = complex_array.imag.tolist()
            interleaved = [val for pair in zip(real_parts, imag_parts) for val in pair]

            timestamp = time.time()
            message = f"{SEQ_NUM};{timestamp};{interleaved}"
            sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))

            # 写入 CSV
            writer.writerow([SEQ_NUM, timestamp, interleaved])
            print(f"Sent seq {SEQ_NUM}, data: {interleaved}")

            SEQ_NUM += 1
            time.sleep(DELAY)

if __name__ == "__main__":
    run_ebi_server()
