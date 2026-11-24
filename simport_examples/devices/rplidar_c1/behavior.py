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
Behaviour of a Slamtec RPLIDAR C1, for rplidar_c1.yaml next to this file.

Serves /<device>/request with the answers a real C1 (firmware 1.01) gives: device info,
health, sample rate and its configuration (Standard and DenseBoost scan modes). After a
scan command it spins up like the real one (about 1.6 s), then publishes `scan_topic`,
resampled to the C1's 480 clockwise beams, at `rate` Hz on /<device>/scan_standard (SCAN)
or /<device>/scan_dense (EXPRESS_SCAN, which the C1 answers with DenseBoost in any mode).

    python3 behavior.py [--ros-args -p scan_topic:=/scan -p serial:=<32 hex digits>]
"""

import math
import struct
import sys
import time

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from simport import fields_of, make_record, run_node, serve

SCAN, FORCE_SCAN, EXPRESS_SCAN, STOP, RESET = 0x20, 0x21, 0x82, 0x25, 0x40
GET_INFO, GET_HEALTH, GET_SAMPLERATE, GET_LIDAR_CONF = 0x50, 0x52, 0x59, 0x84

MODEL, FIRMWARE_MINOR, FIRMWARE_MAJOR, HARDWARE = 0x41, 1, 1, 18
BEAMS = 480            # samples per revolution: 0.75 degrees apart, clockwise from the front
SPIN_UP = 1.6          # s from the scan command to the first data, as measured on a C1
QUALITY = 52           # the quality a C1 reports for a return

# Mode-independent configuration (type -> value), as a C1 answers it.
CONF = {
    0x70: struct.pack('<H', 2),             # number of scan modes
    0x7C: struct.pack('<H', 1),             # typical scan mode: DenseBoost
    0x01: struct.pack('<HH', 600, 300),
    0x02: struct.pack('<I', 3),
    0x60: struct.pack('<I', 40 * 256),
}
# Per-mode configuration: (name, answer type, us per sample Q8, max distance in m Q8).
MODES = [(b'Standard\0', 0x81, 200 * 256, 16 * 256),
         (b'DenseBoost\0', 0x85, 200 * 256, 40 * 256)]


class RplidarC1(Node):
    """Serves /<device>/request and streams while scanning."""

    def __init__(self):
        super().__init__('rplidar_c1')
        device = self.declare_parameter('device', 'rplidar_c1').value
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        rate = self.declare_parameter('rate', 10.0).value
        serial = self.declare_parameter('serial', '5349'.ljust(32, '0')).value
        self.serial = bytes.fromhex(serial)
        if len(self.serial) != 16:
            raise ValueError('serial: 32 hex digits')
        self.latest = None
        self.mode = None            # 'standard' or 'dense' while scanning
        self.streaming_from = 0.0
        self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.out = {'standard': self.create_publisher(LaserScan, f'/{device}/scan_standard', 10),
                    'dense': self.create_publisher(LaserScan, f'/{device}/scan_dense', 10)}
        serve(self, f'/{device}/request', self.on_request)
        self.create_timer(1.0 / rate, self.tick)

    def on_scan(self, msg):
        self.latest = msg

    def tick(self):
        if self.mode and self.latest is not None and time.monotonic() >= self.streaming_from:
            self.out[self.mode].publish(self.c1_beams(self.latest))

    @staticmethod
    def c1_beams(msg):
        """Resample the scan to the C1's beams (k * 0.75 deg clockwise); quality in intensities."""
        out = LaserScan()
        out.header = msg.header
        out.range_min, out.range_max = msg.range_min, msg.range_max
        n = len(msg.ranges)
        for k in range(BEAMS):
            angle = -2.0 * math.pi * k / BEAMS            # clockwise -> ROS counter-clockwise
            i = round((angle - msg.angle_min) / msg.angle_increment) if msg.angle_increment else 0
            r = msg.ranges[i % n] if n else float('inf')
            hit = math.isfinite(r) and msg.range_min <= r <= msg.range_max
            out.ranges.append(r if hit else float('inf'))
            out.intensities.append(float(QUALITY if hit else 0))
        return out

    def start(self, mode):
        self.mode = mode
        self.streaming_from = time.monotonic() + SPIN_UP

    def on_request(self, request, response):
        req = fields_of(request.request)
        cmd = req['cmd']
        reply = response.replies.append
        if cmd == GET_INFO:
            data = bytes([MODEL, FIRMWARE_MINOR, FIRMWARE_MAJOR, HARDWARE]) + self.serial
            reply(make_record('answer', type=0x04, data=data))
        elif cmd == GET_HEALTH:
            reply(make_record('answer', type=0x06, data=bytes(3)))
        elif cmd == GET_SAMPLERATE:
            reply(make_record('answer', type=0x15, data=struct.pack('<HH', 200, 200)))
        elif cmd == GET_LIDAR_CONF:
            reply(make_record('answer', type=0x20, data=self.conf(req['payload'])))
        elif cmd in (SCAN, FORCE_SCAN):
            reply(make_record('stream_start', size=5, type=0x81))
            self.start('standard')
        elif cmd == EXPRESS_SCAN:      # the C1 streams DenseBoost capsules whatever the mode
            reply(make_record('stream_start', size=84, type=0x85))
            self.start('dense')
        elif cmd in (STOP, RESET):
            self.mode = None
            if cmd == RESET:
                reply(make_record('banner'))
        return response                # motor and unknown commands: no answer, as on a C1

    @staticmethod
    def conf(payload):
        """Answer a configuration request: the type, then its value (none for unknown types)."""
        kind = struct.unpack_from('<I', payload)[0] if len(payload) >= 4 else 0
        head = struct.pack('<I', kind)
        if kind in CONF:
            return head + CONF[kind]
        mode = struct.unpack_from('<H', payload, 4)[0] if len(payload) >= 6 else 0
        if mode < len(MODES):
            name, answer, us, distance = MODES[mode]
            value = {0x7F: name, 0x75: bytes([answer]), 0x71: struct.pack('<I', us),
                     0x74: struct.pack('<I', distance)}.get(kind, b'')
            return head + value
        return head


def main():
    """Run the node."""
    return run_node(RplidarC1)


if __name__ == '__main__':
    sys.exit(main())
