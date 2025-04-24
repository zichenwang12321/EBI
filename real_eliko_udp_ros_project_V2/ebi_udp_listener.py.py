#!/usr/bin/env python3
import socket
import rospy
from std_msgs.msg import Float32MultiArray
import ast
import time
import csv
import os

# 初始化 CSV 路径
save_dir = os.path.expanduser("~/ebi_data_logs")
os.makedirs(save_dir, exist_ok=True)
csv_path = os.path.join(save_dir, "ebi_udp_log.csv")

# UDP 配置
UDP_IP = "192.168.1.134"
UDP_PORT = 1234

# 全局缓存
data_buffer = []
seq_counter = 0

def save_to_csv(buffer):
    if not buffer:
        return
    with open(csv_path, 'a', newline='') as f:
        writer = csv.writer(f, delimiter=';')
        for entry in buffer:
            writer.writerow(entry)

def impedance_udp_receiver():
    global seq_counter, data_buffer

    rospy.init_node('ebi_udp_listener', anonymous=True)
    pub = rospy.Publisher('/eliko/impedance_data', Float32MultiArray, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    rospy.loginfo("Listening for EBI UDP data on %s:%d", UDP_IP, UDP_PORT)

    # 创建 CSV 文件并写入表头
    if not os.path.exists(csv_path):
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f, delimiter=';')
            writer.writerow(['seq', 'timestamp', 'data'])

    last_save_time = time.time()

    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(2048)
            decoded = data.decode('utf-8').strip()

            parts = decoded.split(';')
            if len(parts) != 3:
                rospy.logwarn("Malformed packet: %s", decoded)
                continue

            seq = int(parts[0])
            timestamp = float(parts[1])
            interleaved = ast.literal_eval(parts[2])  # 安全解析字符串列表

            # ROS 发布
            msg = Float32MultiArray()
            msg.data = interleaved
            pub.publish(msg)

            # 加入缓存队列
            data_buffer.append([seq, timestamp, interleaved])
            seq_counter += 1

            # 每秒写入 CSV
            now = time.time()
            if now - last_save_time >= 1.0:
                save_to_csv(data_buffer)
                data_buffer = []
                last_save_time = now

        except Exception as e:
            rospy.logwarn("Error: %s", str(e))
            continue

if __name__ == '__main__':
    try:
        impedance_udp_receiver()
    except rospy.ROSInterruptException:
        pass
