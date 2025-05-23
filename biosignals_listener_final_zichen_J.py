#!/usr/bin/env python
import rospy
import socket
import threading
import csv
import os
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
from scipy.interpolate import interp1d
from datetime import datetime
from dateutil import tz

from teleop_msgs.msg import state
from teleop_msgs.msg import robot_state

# IP & 端口配置
EMG_UDP_IP = "192.168.1.134"
EMG_UDP_PORT = 1234
EBI_UDP_IP = "192.168.1.134"
EBI_UDP_PORT = 1235

# 保存路径
SAVE_FOLDER = f'{os.getcwd()}/biosignals_log/{datetime.now(tz.tzlocal()).strftime("%Y_%m_%H_%M_%S")}'
SAVE_FILENAME_EBI = f'EBI_data.csv'
SAVE_FILENAME_EMG = f'EMG_data.csv'
SAVE_FILENAME_ROBOT = f'robot_data.csv'
SAVE_RATE_HZ = 100  # 保存频率（EMG采样频率）

# 数据缓存和锁
emg_buffer = []
ebi_buffer = []
lock = threading.Lock()
MAX_BUFFER_LEN = 1000

save_path_EBI = os.path.join(SAVE_FOLDER, SAVE_FILENAME_EBI)
save_path_EMG = os.path.join(SAVE_FOLDER, SAVE_FILENAME_EMG)
save_path_robot = os.path.join(SAVE_FOLDER, SAVE_FILENAME_ROBOT)

if not os.path.exists(SAVE_FOLDER):
    os.makedirs(SAVE_FOLDER)


csv_file_EBI = open(save_path_EBI, mode='w', newline='')
csv_writer_EBI = csv.writer(csv_file_EBI)
header_EBI = ['system_time', 'source_timestamp'] + \
            ['ebi_amp_{}'.format(i+1) for i in range(15)]
csv_writer_EBI.writerow(header_EBI)

csv_file_EMG = open(save_path_EMG, mode='w', newline='')
csv_writer_EMG = csv.writer(csv_file_EMG)
header_EMG = ['system_time', 'source_timestamp'] + \
        ['emg_{}'.format(i+1) for i in range(16)]
csv_writer_EMG.writerow(header_EMG)

csv_file_robot = open(save_path_robot, mode='w', newline='')
csv_writer_robot = csv.writer(csv_file_robot)
header_robot = ['system_time', 'source_timestamp'] + \
    ["robot_pos_x","robot_pos_y","robot_pos_z"] + \
    ["robot_vel_x","robot_vel_y","robot_vel_z"] + \
    ["robot_force_x","robot_force_y","robot_force_z", "robot_torque_x", "robot_torque_y", "robot_torque_z"] + \
    ['joint_pos_{}'.format(i+1) for i in range(7)] + \
    ['joint_vel_{}'.format(i+1) for i in range(7)]
    # ['robot_wrench_{}'.format(i+1) for i in range(6)] + \
csv_writer_robot.writerow(header_robot)

row_EMG = []
row_EBI = []
row_robot = []


def parse_udp_message(data_str):
    try:
        parts = data_str.strip().split(';')
        if len(parts) < 3:
            return None, None
        timestamp = float(parts[1]) - 60*60*2 # denmark summer time dependant
        data_values = list(map(float, parts[2].split()))
        return timestamp, data_values
    except Exception as e:
        rospy.logwarn('Failed to parse UDP message: {}'.format(e))
        return None, None

def parse_udp_message_emg(data_str):
    try:
        message_list = [0.0] * 16
        parts = data_str.strip().split(';')
        seq,ts,*emg_data = parts
        for i in range(min(len(emg_data), 16)):
            message_list[i] = float(emg_data[i])
        return [float(ts)] + message_list  # 加入系统时间作为时间戳
    except Exception as e:
        rospy.logwarn('Failed to parse EMG message: {}'.format(e))
        return None

def emg_listener():
    global row_EMG
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((EMG_UDP_IP, EMG_UDP_PORT))
    rospy.loginfo('Listening EMG UDP on port {}'.format(EMG_UDP_PORT))
    sock.settimeout(1.0)
    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(1024)
            data_str = data.decode('utf-8')
            # print(data_str)
            values = parse_udp_message_emg(data_str)
            if values:
                
                row_EMG = [time.time()] + values
                csv_writer_EMG.writerow(row_EMG)
                csv_file_EMG.flush()
                # with lock:
                #     emg_buffer.append(values)
                #     if len(emg_buffer) > MAX_BUFFER_LEN:
                #         emg_buffer.pop(0)
        except Exception as e:
            rospy.logwarn('EMG socket error: {}'.format(e))

def ebi_listener():
    global row_EBI
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((EBI_UDP_IP, EBI_UDP_PORT))
    rospy.loginfo('Listening EBI UDP on port {}'.format(EBI_UDP_PORT))
    sock.settimeout(1.0)
    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(4096)
            data_str = data.decode('utf-8')
            ts, values = parse_udp_message(data_str)
            if ts is not None and values:
                if len(values) < 30:
                    values += [0.0] * (30 - len(values))
                real = np.array(values[:15])
                imag = np.array(values[15:30])
                amplitude = np.sqrt(real**2 + imag**2)

                row_EBI = [time.time(), ts] + amplitude.tolist()
                csv_writer_EBI.writerow(row_EBI)
                csv_file_EBI.flush()
                    
                    # ebi_buffer.append((ts, amplitude.tolist()))
                    # if len(ebi_buffer) > MAX_BUFFER_LEN:
                    #     ebi_buffer.pop(0)
        except Exception as e:
            rospy.logwarn('EBI socket error: {}'.format(e))


def get_robot_state(msg : robot_state):
    global row_robot
    robot_pos =  [msg.state.position.x, msg.state.position.y, msg.state.position.z]
    robot_vel =  [msg.state.twist.linear.x, msg.state.twist.linear.y, msg.state.twist.linear.z]
    robot_wrench =  [msg.state.wrench.force.x, msg.state.wrench.force.y, msg.state.wrench.force.z, msg.state.wrench.torque.x, msg.state.wrench.torque.y, msg.state.wrench.torque.z]
    joint_pos = list(msg.q.data)
    joint_vel = list(msg.qdot.data)
    
    row_robot = [time.time(), msg.time.data] + robot_pos + robot_vel + robot_wrench + joint_pos + joint_vel
    csv_writer_robot.writerow(row_robot)
    csv_file_robot.flush()



def main():
    rospy.init_node('biosignals_listener_node')
    pub = rospy.Publisher('/biosignals', Float64MultiArray, queue_size=1)
    sub = rospy.Subscriber('/robot_state', robot_state, get_robot_state)

    threading.Thread(target=emg_listener, daemon=True).start()
    threading.Thread(target=ebi_listener, daemon=True).start()

    rate = rospy.Rate(SAVE_RATE_HZ)
    while not rospy.is_shutdown():
        # with lock:
            # if len(ebi_buffer) < 2: #  if len(emg_buffer) < 2 or len(ebi_buffer) < 2
            #     rate.sleep()
            #     continue

            # try:
                # 提取 EBI 时间戳和幅值数组
                # ebi_times = [x[0] for x in ebi_buffer]
                # ebi_data_array = np.array([x[1] for x in ebi_buffer])  # shape: (N, 15)

                # # 为每个频率通道建立插值函数
                # # ebi_interp_funcs = [
                # #     interp1d(ebi_times, ebi_data_array[:, i], bounds_error=False, fill_value='extrapolate')
                # #     for i in range(15)
                # # ]
                # # 为每个频率通道建立插值函数，超出范围则线性外推，但后续会限制为非负
                # ebi_interp_funcs = []
                # for i in range(15):
                #     interp_func = interp1d(
                #         ebi_times,
                #         ebi_data_array[:, i],
                #         kind='linear',
                #         bounds_error=False,
                #         fill_value='extrapolate'
                #     )
                #     ebi_interp_funcs.append(interp_func)

                # # 使用最新 EMG 时间戳对齐
                # latest_emg_sample = emg_buffer[-1]
                # target_time = latest_emg_sample[0]
                # aligned_emg = latest_emg_sample[1:]

                # 插值计算 EBI 对应时间的幅值
                # aligned_ebi = [float(f(target_time)) for f in ebi_interp_funcs]
                # 边界检查避免外推过远
                # if target_time < min(ebi_times) or target_time > max(ebi_times):
                #     rospy.logwarn("Target EMG timestamp {:.4f} out of EBI range [{:.4f}, {:.4f}]".format(
                #         target_time, min(ebi_times), max(ebi_times)))

                # 插值计算并限制为非负
                # aligned_ebi = [max(0.0, float(f(target_time))) for f in ebi_interp_funcs]

                # # 写入CSV
                # row_EBI = [time.time(), ebi_times] +  aligned_ebi
                # csv_writer_EBI.writerow(row_EBI)
                # csv_file_EBI.flush()
                # row_EMG = [time.time(), target_time] + aligned_emg
                # csv_writer_EMG.writerow(row_EMG)
                # csv_file_EMG.flush()
                # row_robot = [time.time(), robot_time] + robot_pos + robot_vel + robot_wrench + joint_pos + joint_vel
                # csv_writer_robot.writerow(row_robot)
                # csv_file_robot.flush()
                

        # 发布ROS消息
        msg = Float64MultiArray()
        msg.data = row_EMG + row_EBI + row_robot
        # print(msg.data)
        pub.publish(msg)

            # except Exception as e:
            #     rospy.logwarn('Interpolation alignment error: {}'.format(e))

        rate.sleep()

    csv_file_EBI.close()
    csv_file_EMG.close()
    csv_file_robot.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
