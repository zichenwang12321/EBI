# Real Eliko Impedance ROS UDP System

This project enables **real-time acquisition**, **UDP transmission**, and **ROS integration** of **Eliko Impedance Sensor** data from a **Windows machine (MATLAB)** to a **Linux machine running ROS**.

## Project Structure

```text
real_eliko_udp_ros_project/
├── eliko_client.py                # Windows-side UDP sender, calls MATLAB
├── ebi_udp_listener.py           # Linux-side ROS subscriber node
├── ElikoSample.m (MATLAB)        # Impedance data sampling logic
├── ElikoRead.m (MATLAB)          # Hardware interfacing layer
├── PicometerControl.mexw64       # Required compiled driver for Eliko
├── code/
│   └── src/teleop_enhancer_pkg/
│       └── Log/ebi/              # Server-side logs (Windows)
└── ~/ebi_data_logs/              # Client-side logs (Linux)
```

## Features

- Real-time impedance data acquisition via MATLAB (`ElikoSample.m`)
- UDP data transmission from Windows to Linux
- ROS1 integration via `/eliko/impedance_data` topic
- CSV logging (both sender & receiver sides)
- Handles both real hardware and simulated test mode (`TEST = True`)
- Supports clean shutdown via `Ctrl+C`

## Requirements

### Windows (Sender)

- MATLAB (with Eliko hardware setup and driver installed)
- MATLAB Engine API for Python (`pip install matlab.engine`)
- Python 3.10+
- Eliko MATLAB scripts: `ElikoSample.m`, `ElikoRead.m`, `PicometerControl.mexw64`

### Linux (Receiver)

- Python 3.10+
- ROS Noetic (or compatible)
- `rospy`, `std_msgs`
- `ebi_udp_listener.py` should be placed in a ROS package (e.g. `teleop_enhancer_pkg`)

## Setup Instructions

### Windows (Real Device)

1. Ensure MATLAB and Eliko are correctly configured.
2. Open a terminal and run:
   ```bash
   python eliko_client.py
   ```
3. (Optional) Set `TEST = True` in `eliko_client.py` to simulate data.

### Linux (ROS Integration)

1. Source your ROS workspace:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```
2. Run the ROS UDP listener:
   ```bash
   rosrun teleop_enhancer_pkg ebi_udp_listener.py
   ```
3. Visualize data:
   ```bash
   rostopic echo /eliko/impedance_data
   ```

## Data Format

Each UDP packet (and CSV entry) is structured as:

```text
seq;timestamp;[real1, imag1, real2, imag2, ...]
```

- `seq`: Sequential packet number
- `timestamp`: UNIX timestamp
- `interleaved`: Real and imaginary parts interleaved

## Logging

### On Windows (Sender):

File: `code/src/teleop_enhancer_pkg/Log/ebi/ebi_data_server.csv`

### On Linux (Receiver):

File: `~/ebi_data_logs/ebi_udp_log.csv`

- New file created if not exists
- Data saved every second

## Stopping the System

- Press `Ctrl+C` on either terminal
- Graceful shutdown is supported via signal handling

## Contact & Support

For integration issues or questions, feel free to reach out.