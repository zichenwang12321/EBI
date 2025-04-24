
#!/usr/bin/env python3
import socket
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

UDP_IP = "0.0.0.0" # 监听本地所有接口(待确定？？)
UDP_PORT = 7070 #

def udp_listener():
    rospy.init_node('eliko_udp_subscriber', anonymous=True)
    pub = rospy.Publisher('/eliko/impedance_data', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(50)  # 50Hz

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)

    rospy.loginfo("Listening for UDP data on port %d", UDP_PORT)
    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(65535)
            floats = np.frombuffer(data, dtype=np.float32)
            msg = Float32MultiArray(data=floats.tolist())
            pub.publish(msg)
        except socket.timeout:
            continue
        rate.sleep()

if __name__ == '__main__':
    try:
        udp_listener()
    except rospy.ROSInterruptException:
        pass
