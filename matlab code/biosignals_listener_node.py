#!/usr/bin/env python
import rospy
import socket
import threading
import csv
import os
import time
from std_msgs.msg import Float32MultiArray

# 记调整IP 和port
EMG_UDP_IP = "192.168.1.13???"#192.168.1.134
EMG_UDP_PORT = 1234

EBI_UDP_IP = "192.168.1.134"
EBI_UDP_PORT = 1235

SAVE_FOLDER = '/home/your_username/biosignals_log/'  # 自己的路径
SAVE_FILENAME = 'biosignals_data.csv'

SAVE_RATE_HZ = 1  # 每秒保存一次

# 全局变量
emg_data = None
emg_timestamp = None

ebi_data = None
ebi_timestamp = None

lock = threading.Lock()

def parse_udp_message(data_str):
    try:
        parts = data_str.strip().split(';')
        if len(parts) < 3:
            return None, None
        seq_num = int(parts[0])
        timestamp = float(parts[1])
        data_values = list(map(float, parts[2].split()))
        return timestamp, data_values
    except Exception as e:
        rospy.logwarn('Failed to parse UDP message: {}'.format(e))
        return None, None

def emg_listener():
    global emg_data, emg_timestamp
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((EMG_UDP_IP, EMG_UDP_PORT))
    rospy.loginfo('Listening EMG UDP on port {}'.format(EMG_UDP_PORT))
    sock.settimeout(1.0)
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(4096)
            data_str = data.decode('utf-8')
            ts, values = parse_udp_message(data_str)
            if ts is not None:
                with lock:
                    emg_data = values
                    emg_timestamp = ts
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logwarn('EMG socket error: {}'.format(e))

def ebi_listener():
    global ebi_data, ebi_timestamp
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((EBI_UDP_IP, EBI_UDP_PORT))
    rospy.loginfo('Listening EBI UDP on port {}'.format(EBI_UDP_PORT))
    sock.settimeout(1.0)
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(4096)
            data_str = data.decode('utf-8')
            ts, values = parse_udp_message(data_str)
            if ts is not None:
                with lock:
                    ebi_data = values
                    ebi_timestamp = ts
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logwarn('EBI socket error: {}'.format(e))

def main():
    global emg_data, emg_timestamp, ebi_data, ebi_timestamp

    rospy.init_node('biosignals_listener_node')

    pub = rospy.Publisher('/biosignals', Float32MultiArray, queue_size=10)

    # 确保保存路径存在
    if not os.path.exists(SAVE_FOLDER):
        os.makedirs(SAVE_FOLDER)
    save_path = os.path.join(SAVE_FOLDER, SAVE_FILENAME)

    # 打开CSV文件
    csv_file = open(save_path, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    header = ['system_timestamp', 'emg_timestamp', 'ebi_timestamp'] + \
             ['emg_{}'.format(i) for i in range(1, 17)] + \
             ['ebi_real_{}'.format(i) for i in range(1, 16)] + \
             ['ebi_imag_{}'.format(i) for i in range(1, 16)]
    csv_writer.writerow(header)

    # 启动两个监听线程
    threading.Thread(target=emg_listener, daemon=True).start()
    threading.Thread(target=ebi_listener, daemon=True).start()

    rate = rospy.Rate(SAVE_RATE_HZ)

    while not rospy.is_shutdown():
        with lock:
            if emg_data is not None and ebi_data is not None:
                # 合并EMG和EBI数据
                system_ts = time.time()

                if len(emg_data) < 16:
                    emg_data_filled = emg_data + [0.0]*(16-len(emg_data))
                else:
                    emg_data_filled = emg_data[:16]

                if len(ebi_data) < 30:
                    ebi_data_filled = ebi_data + [0.0]*(30-len(ebi_data))
                else:
                    ebi_data_filled = ebi_data[:30]

                full_row = [system_ts, emg_timestamp, ebi_timestamp] + emg_data_filled + ebi_data_filled
                csv_writer.writerow(full_row)
                csv_file.flush()

                # 发布到ROS话题
                msg = Float32MultiArray()
                msg.data = emg_data_filled + ebi_data_filled
                pub.publish(msg)

        rate.sleep()

    csv_file.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
