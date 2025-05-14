#!/usr/bin/env python
import rospy
import socket
import threading
import csv
import os
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from scipy.interpolate import interp1d

# 记调整IP 和port
EMG_UDP_IP = "192.168.1.134"  # 监听所有网卡
EMG_UDP_PORT = 1234
EBI_UDP_IP = "192.168.1.134"
EBI_UDP_PORT = 1235

# 保存路径
SAVE_FOLDER = '/home/panda2/zichen/biosignals_log/'  # 改成你自己的用户名路径
SAVE_FILENAME = 'biosignals_data.csv'
SAVE_RATE_HZ = 100  # 目标频率（与EMG采样同步）

# 全局数据缓存
emg_buffer = []
ebi_buffer = []
lock = threading.Lock()

# 最大缓冲区长度（假设最多缓存10秒）
MAX_BUFFER_LEN = 1000

def parse_udp_message(data_str):
    try:
        parts = data_str.strip().split(';')
        if len(parts) < 3:
            return None, None
        timestamp = float(parts[1])
        data_values = list(map(float, parts[2].split()))
        return timestamp, data_values
    except Exception as e:
        rospy.logwarn('Failed to parse UDP message: {}'.format(e))
        return None, None

def emg_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((EMG_UDP_IP, EMG_UDP_PORT))
    rospy.loginfo('Listening EMG UDP on port {}'.format(EMG_UDP_PORT))
    sock.settimeout(1.0)
    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(4096)
            data_str = data.decode('utf-8')
            ts, values = parse_udp_message(data_str)
            if ts is not None:
                with lock:
                    if len(values) < 16:
                        values += [0.0] * (16 - len(values))
                    emg_buffer.append((ts, values[:16]))
                    if len(emg_buffer) > MAX_BUFFER_LEN:
                        emg_buffer.pop(0)
        except Exception as e:
            rospy.logwarn('EMG socket error: {}'.format(e))

def ebi_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((EBI_UDP_IP, EBI_UDP_PORT))
    rospy.loginfo('Listening EBI UDP on port {}'.format(EBI_UDP_PORT))
    sock.settimeout(1.0)
    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(4096)
            data_str = data.decode('utf-8')
            ts, values = parse_udp_message(data_str)
            if ts is not None:
                with lock:
                    if len(values) < 30:
                        values += [0.0] * (30 - len(values))
                    real = np.array(values[:15])
                    imag = np.array(values[15:30])
                    amplitude = np.sqrt(real**2 + imag**2)
                    ebi_buffer.append((ts, amplitude.tolist()))
                    if len(ebi_buffer) > MAX_BUFFER_LEN:
                        ebi_buffer.pop(0)
        except Exception as e:
            rospy.logwarn('EBI socket error: {}'.format(e))

def main():
    rospy.init_node('biosignals_listener_node')
    pub = rospy.Publisher('/biosignals', Float32MultiArray, queue_size=10)

    # 创建保存路径
    if not os.path.exists(SAVE_FOLDER):
        os.makedirs(SAVE_FOLDER)
    save_path = os.path.join(SAVE_FOLDER, SAVE_FILENAME)

    csv_file = open(save_path, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    header = ['system_time', 'aligned_timestamp'] + \
             ['emg_{}'.format(i+1) for i in range(16)] + \
             ['ebi_amp_{}'.format(i+1) for i in range(15)]
    csv_writer.writerow(header)

    # 启动线程
    threading.Thread(target=emg_listener, daemon=True).start()
    threading.Thread(target=ebi_listener, daemon=True).start()

    rate = rospy.Rate(SAVE_RATE_HZ)
    while not rospy.is_shutdown():
        with lock:
            if len(emg_buffer) < 2 or len(ebi_buffer) < 2:
                rate.sleep()
                continue

            # 获取最新时间戳范围
            latest_emg_ts = emg_buffer[-1][0]
            emg_times = [item[0] for item in emg_buffer]
            ebi_times = [item[0] for item in ebi_buffer]

            # 对EBI做插值，获取与EMG时间对齐的数据
            try:
                ebi_data_array = np.array([x[1] for x in ebi_buffer])
                interp_funcs = [interp1d(ebi_times, ebi_data_array[:, i], bounds_error=False, fill_value='extrapolate')
                                for i in range(15)]
                aligned_ebi = [float(f(latest_emg_ts)) for f in interp_funcs]

                # 找到最近的EMG
                aligned_emg = emg_buffer[-1][1]

                # 写入CSV
                row = [time.time(), latest_emg_ts] + aligned_emg + aligned_ebi
                csv_writer.writerow(row)
                csv_file.flush()

                # 发布ROS消息
                msg = Float32MultiArray()
                msg.data = aligned_emg + aligned_ebi
                pub.publish(msg)

            except Exception as e:
                rospy.logwarn('Interpolation error: {}'.format(e))

        rate.sleep()

    csv_file.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
