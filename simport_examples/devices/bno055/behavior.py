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
Behaviour of a Bosch BNO055 in UART mode, for bno055.yaml next to this file.

Serves /<device>/registers, where simport sends the driver's register reads and writes.
Writes are stored and acknowledged, reads return the stored bytes, and a read of the 45 data
registers (0x08..0x34) is answered with the device file's `data` message, which simport fills
from the latest IMU message.

    python3 behavior.py [--ros-args -p device:=bno055]
"""

import sys

from rclpy.node import Node
from simport import fields_of, make_record, run_node, serve


DATA_START, DATA_LENGTH = 0x08, 45
WRITE_OK = 0x01


class Bno055(Node):
    """Serves /<device>/registers."""

    def __init__(self):
        super().__init__('bno055')
        device = self.declare_parameter('device', 'bno055').value
        self.registers = bytearray(128)
        self.registers[0x00] = 0xA0  # chip id
        self.registers[0x35] = 0xFF  # calibration status: sys, gyro, accel, mag all calibrated
        serve(self, f'/{device}/registers', self.on_request)

    def on_request(self, request, response):
        req = fields_of(request.request)
        reg = req['reg']
        if request.request.name == 'write':
            data = req['data']
            self.registers[reg:reg + len(data)] = data
            response.replies.append(make_record('write_response', status=WRITE_OK))
        elif reg == DATA_START and req['length'] == DATA_LENGTH:
            response.replies.append(make_record('data'))
        else:
            data = bytes(self.registers[reg:reg + req['length']])
            response.replies.append(make_record('read_response', data=data))
        return response


def main():
    """Run the node."""
    return run_node(Bno055)


if __name__ == '__main__':
    sys.exit(main())
