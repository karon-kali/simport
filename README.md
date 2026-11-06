# simport

[![CI](https://github.com/karon-kali/simport/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/karon-kali/simport/actions/workflows/ci.yml)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E)](https://docs.ros.org/en/jazzy/)
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

**Run your real robot drivers against simulation.**

Built and tested on ROS 2 Jazzy (Ubuntu 24.04); the CI badge shows the current state of `main`.

simport puts simulated sensors and motors on emulated hardware ports (serial ports, UDP and
TCP sockets, CAN buses) speaking the device's real protocol. Your unmodified driver opens
`/tmp/simport/imu0` or talks to `127.0.0.2:5600` exactly as it would talk to the hardware.

```
simulator ──ROS topics──▶ simport ──packets──▶ /tmp/simport/imu0 ──▶ your real driver
          ◀──topics / services── ◀──packets── commands ◀───────────┘
```

simport translates and does not decide. You describe the packets in YAML: their fields,
constants, counters, lengths and checksums, and where each value comes from. simport turns ROS
messages into packets, and the driver's packets into ROS messages or service calls. Whatever
the device decides, such as answers, modes or registers, lives in a small ROS node of yours.

## A device in YAML

```yaml
device: imu0
port: { serial: /tmp/simport/imu0, baud: 115200 }

send:
  - name: imu_data
    from: { topic: /imu, type: sensor_msgs/msg/Imu }
    rate: 200
    fields:
      - sync: { u16: 0xAA55, endian: big }
      - seq:  { u8, counter: true }
      - gx:   { i16, from: angular_velocity.x, scale: 1000.0 }   # rad/s -> mrad/s
      - az:   { f32, from: linear_acceleration.z }
      - crc:  { u16, checksum: crc16_ccitt_false, over: seq }
```

```bash
ros2 run simport_ros simport_check imu0.yaml          # validate it, see the exact bytes
ros2 launch simport_ros device.launch.py config_file:=$PWD/imu0.yaml
```

Start from [templates/device.yaml](templates/device.yaml), a commented device that runs as
is, and [templates/device_node.py](templates/device_node.py), the node that answers the
driver's commands.

## What it handles

- **Ports**: serial (pseudo-terminals, paced to the baud rate, hot-plug like hardware), UDP
  with any `127.x.y.z` device address, TCP servers with per-client streams, SocketCAN and
  CAN-FD.
- **Binary packets**: integers, floats, endianness, constants, counters, bit fields, variable
  length, a shared envelope (start bytes, lengths, byte stuffing), CRC-8 to CRC-64 presets,
  custom CRCs, sums, XOR and Fletcher.
- **Arrays and point clouds**: one message becomes many packets (a lidar revolution, a range
  image), with arrays transposed, reversed or resampled; 3-D lidars straight from
  `PointCloud2`.
- **Text**: printf-style formats, NMEA positions, times and checksums.
- **Timing**: each device sends at its own rate on the simulation clock, and goes silent when
  the simulation stops.
- **Two-way devices**: the driver's packets become topics or service calls; the service's
  answers are sent back to whoever asked.
- **Tools**: `simport_check`, `/diagnostics`, and raw packets on a topic.
- **Plugins** for what YAML cannot describe: codecs, ports, framers, data sources.

## Examples

Each example is a real device, in its own folder in `simport_examples/devices/`: its device
file(s), and, where the device decides something, the behaviour that answers the driver.

| Example | Port | Behaviour | Shows |
|---|---|---|---|
| `rplidar_a1` | `/tmp/simport/rplidar0` | `behavior.py`: device info, health, scan modes, scanning between SCAN and STOP | Slamtec RPLIDAR A1, scans from a `LaserScan` |
| `bno055` | `/tmp/simport/bno055` | `behavior.py`: the register map | Bosch BNO055 in UART mode, data from an `Imu` |
| `nmea_gps` | `/tmp/simport/gps0` | none: it only streams | NMEA GGA + RMC at 5 Hz on a 9600-baud line |
| `ouster_os1_64` | `127.0.0.5`: HTTP `8080`, UDP to `7502`/`7503` | `behavior.py`: the HTTP API | Ouster OS1-64, lidar packets from a `PointCloud2`, IMU packets |
| `dynamixel` | `/tmp/simport/dxl0` | `behavior.py`: control tables, all read/write instructions | two Dynamixel XM430 motors (Protocol 2.0) driving simulated joints |
| `damiao` | CAN-FD on `vcan0` | `damiao_bus.cpp`, in process: MIT and velocity control, parameters | seven Damiao motors laid out like an OpenArm arm |

```bash
ros2 launch simport_examples example.launch.py example:=rplidar_a1
```

`example:=<name>` starts every device file in the folder, and its `behavior.py` if it has one.
Behaviours that run in process (Damiao) are declared in the device file. Your own device file
runs with `ros2 launch simport_ros device.launch.py config_file:=/path/to/device.yaml`; to make
one, copy a device folder, or start from the [templates](templates): a device file,
[a behaviour node](templates/device_node.py), or
[a behaviour plugin](templates/behavior_plugin) for fast control loops.

### With real drivers

Point the device's unmodified ROS 2 driver at the emulated port:

| Example | Driver | Settings |
|---|---|---|
| `rplidar_a1` | `rplidar_ros` | `ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/tmp/simport/rplidar0 -p serial_baudrate:=115200` |
| `bno055` | `bno055` | `connection_type: uart`, `uart_port: /tmp/simport/bno055` |
| `ouster_os1_64` | `ouster_ros` | `ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=127.0.0.5:8080 lidar_port:=7502 imu_port:=7503` |
| `nmea_gps` | `nmea_navsat_driver` | `port: /tmp/simport/gps0`, `baud: 9600` |
| `dynamixel` | `dynamixel_hardware_interface` | `port_name: /tmp/simport/dxl0`, `baud_rate: 1000000`, IDs 11 and 12 |
| `damiao` | `openarm_hardware` | `can_interface: vcan0`, `can_fd: true` |

Every example has been run in Gazebo with its unmodified driver, comparing what the driver
reports with what the simulator produced: lidar points within 1.4 cm (RPLIDAR) and 0.3 mm
(Ouster), exact IMU rates, GPS within a centimetre, and motor positions within one encoder
step. `openarm_hardware` runs at its full 750 Hz, with replies in under a millisecond.

The motor examples drive joints: they publish commands on `/sim/<joint>/cmd_pos` and read
`/sim/joint_states`. The Damiao example needs a virtual CAN-FD bus once:
`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan mtu 72 && sudo ip link set up vcan0`.

### With a simulator

simport reads and writes ordinary ROS 2 topics, so any simulator works. For Gazebo, bridge its
topics with `ros_gz_bridge` to the ones the devices use (`/imu`, `/scan`, `/sim/lidar/points`,
`/sim/joint_states`, `/sim/<joint>/cmd_pos`) and run with `use_sim_time:=true`.

- Bridge a 3-D `gpu_lidar` as its `PointCloud2` topic; the LaserScan bridge keeps only one row.
- Gazebo's IMU sensors publish only if the world loads the `gz-sim-imu-system` plugin.
- For motors, give each joint a `JointPositionController` on `/sim/<joint>/cmd_pos` and the
  model a `JointStatePublisher`.

## Documentation

- [Device configuration reference](docs/configuration.md)
- [Writing a plugin](docs/writing_plugins.md)
- [Templates](templates): a device file, a device node, codec and behavior plugin packages

## Packages

| Package | Contents |
|---|---|
| `simport_core` | ROS-agnostic core: data model, configuration, pipelines, plugin interfaces |
| `simport_plugins` | Ports, binary/text/HTTP codecs, framers |
| `simport_ros` | The device node, ROS sources, `simport_check`, the `simport` Python module |
| `simport_msgs` | `Record` and `Frame` messages, the `Exchange` service |
| `simport_examples` | Example devices, one folder each: device files and their behaviours |

## Building

Requires ROS 2 Jazzy.

```bash
mkdir -p ~/simport_ws/src && cd ~/simport_ws/src
git clone https://github.com/karon-kali/simport.git
cd .. && rosdep install --from-paths src -y --ignore-src
colcon build
colcon test && colcon test-result --verbose
```

## License

Apache-2.0. See [LICENSE](LICENSE).
