^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simport_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-06)
------------------
* One folder per device in ``devices/``: its device files, its behaviour (``behavior.py``, or
  an in-process plugin) and its data; ``example.launch.py example:=<folder>`` runs it
* Devices, each run against Gazebo with its unmodified driver: Slamtec RPLIDAR A1
  (rplidar_ros), Bosch BNO055 in UART mode (bno055), NMEA GPS (nmea_navsat_driver), Ouster
  OS1-64 firmware 3.2 (ouster_ros), Dynamixel XM430 pan-tilt (dynamixel_hardware_interface),
  Damiao motors laid out like an OpenArm arm (openarm_hardware, at its full 750 Hz)
* Removed the invented-protocol examples (generic IMU, rangefinder), the driver scripts and
  the end-to-end launch test

0.1.0 (2026-11-05)
------------------
* First release.
* Example devices: generic IMU, NMEA GPS, rangefinder (serial and UDP)
* Driver scripts and end-to-end launch tests
