#!/usr/bin/env python3
# Copyright 2026 simport contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Behaviour of a Slamtec RPLIDAR A1, for rplidar_a1.yaml next to this file.

Serves /<device>/request, where simport sends the requests of the Slamtec SDK: device info,
health and the scan-mode configuration are answered as an A1M8 (firmware 1.29) would. Between
SCAN and STOP, `scan_topic` is republished on /<device>/scan at `rate` Hz, which simport sends
to the driver as standard-scan samples.

    python3 behavior.py [--ros-args -p device:=rplidar0 -p scan_topic:=/scan]
"""

import struct
import sys

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from simport import fields_of, make_record, run_node, serve


GET_INFO, GET_HEALTH, SCAN, FORCE_SCAN, STOP, RESET = 0x50, 0x52, 0x20, 0x21, 0x25, 0x40
GET_LIDAR_CONF = 0x84

MODEL_A1M8, FIRMWARE_1_29, HARDWARE = 0x18, 0x011D, 7
STANDARD_MODE = 0
# GET_LIDAR_CONF answers for scan mode 0, "Standard": configuration type -> value.
CONF = {
    0x7C: struct.pack('<H', STANDARD_MODE),     # typical scan mode
    0x71: struct.pack('<I', 500 * 256),         # us per sample, Q8
    0x74: struct.pack('<I', 12 * 256),          # max distance in m, Q8
    0x75: bytes([0x81]),                        # answer type: standard samples
    0x7F: b'Standard\0',                        # mode name
    0x01: struct.pack('<HH', 330, 660),         # motor rpm, pwm
}


class RplidarA1(Node):
    """Serves /<device>/request and streams while scanning."""

    def __init__(self):
        super().__init__('rplidar_a1')
        device = self.declare_parameter('device', 'rplidar0').value
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        rate = self.declare_parameter('rate', 5.5).value  # revolutions per second
        self.latest = None
        self.scanning = False
        self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.stream = self.create_publisher(LaserScan, f'/{device}/scan', 10)
        serve(self, f'/{device}/request', self.on_request)
        self.create_timer(1.0 / rate, self.tick)

    def on_scan(self, msg):
        self.latest = msg

    def tick(self):
        if self.scanning and self.latest is not None:
            self.stream.publish(self.latest)

    def on_request(self, request, response):
        req = fields_of(request.request)
        cmd = req['cmd']
        reply = response.replies.append
        if cmd == GET_INFO:
            data = struct.pack('<BHB', MODEL_A1M8, FIRMWARE_1_29, HARDWARE) + bytes(16)
            reply(make_record('answer', type=0x04, data=data))
        elif cmd == GET_HEALTH:
            reply(make_record('answer', type=0x06, data=bytes(3)))  # good, no error code
        elif cmd in (SCAN, FORCE_SCAN):
            reply(make_record('stream_start', size=5, type=0x81))
            self.scanning = True
        elif cmd in (STOP, RESET):
            self.scanning = False
        elif cmd == GET_LIDAR_CONF:
            conf = struct.unpack('<I', req['payload'][:4])[0]
            if conf in CONF:
                reply(make_record('answer', type=0x20, data=req['payload'][:4] + CONF[conf]))
        return response


def main():
    """Run the node."""
    return run_node(RplidarA1)


if __name__ == '__main__':
    sys.exit(main())
