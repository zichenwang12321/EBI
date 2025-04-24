import matlab.engine
import socket
import numpy as np
import time

def get_impedance_complex(eng):
    try:
        complex_mat = eng.ElikoSample()
        complex_array = np.array(complex_mat._data).reshape(complex_mat.size, order='F')
        return complex_array
    except Exception as e:
        print("MATLAB error:", e)
        return None

def send_udp_packet(data, ip="192.168.1.10", port=7070):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    flattened = np.empty((data.size * 2,), dtype=np.float32)
    flattened[0::2] = data.real.flatten()
    flattened[1::2] = data.imag.flatten()
    sock.sendto(flattened.tobytes(), (ip, port))
    sock.close()

if __name__ == "__main__":
    eng = matlab.engine.start_matlab()
    eng.cd(r'C:/Users/王子辰/Desktop/zichen/real_eliko_udp_ros_project', nargout=0)
    while True:
        data = get_impedance_complex(eng)
        if data is not None:
            send_udp_packet(data)
        time.sleep(0.05)  # ~20Hz 发送速率
