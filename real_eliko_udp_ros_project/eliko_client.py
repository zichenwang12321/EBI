
import matlab.engine
import socket
import numpy as np

def get_impedance_complex():
    eng = matlab.engine.start_matlab()
    eng.cd(r'C:/Users/王子辰/Desktop/zichen/real_eliko_udp_ros_project', nargout=0)
    try:
        complex_mat = eng.ElikoSample()
        complex_array = np.array(complex_mat._data).reshape(complex_mat.size, order='F')
    except Exception as e:
        print("MATLAB error:", e)
        return None
    return complex_array

def send_udp_packet(data, ip="192.168.1.10", port=7070):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    flattened = np.empty((data.size * 2,), dtype=np.float32)
    flattened[0::2] = data.real.flatten()
    flattened[1::2] = data.imag.flatten()
    sock.sendto(flattened.tobytes(), (ip, port))

if __name__ == "__main__":
    data = get_impedance_complex()
    if data is not None:
        send_udp_packet(data)
