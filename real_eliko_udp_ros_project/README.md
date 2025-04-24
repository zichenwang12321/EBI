
# Real Eliko Impedance ROS UDP System

## Description

This project enables communication between a Windows machine running Eliko impedance acquisition and a Linux ROS machine for real-time integration.

## Components

- `eliko_client.py`: Calls MATLAB ElikoSample and sends data via UDP (Windows).
- `eliko_udp_ros_subscriber.py`: ROS node to receive UDP and publish to `/eliko/impedance_data` (Linux).
- MATLAB Folder: Must include `ElikoSample.m`, `ElikoRead.m`, `PicometerControl.mexw64` etc.

## Usage

### Windows

1. Ensure MATLAB Engine for Python is installed and working.
2. Run `eliko_client.py`.

### Linux

```bash
rosrun <your_package> eliko_udp_ros_subscriber.py
rostopic echo /eliko/impedance_data
```

## Requirements

- MATLAB with Eliko device connected
- Python 3.10
- ROS Noetic or equivalent
