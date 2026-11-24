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

"""The C1 behaviour answers exactly what a real RPLIDAR C1 (firmware 1.01) answers."""

import importlib.util
import os
import struct

_spec = importlib.util.spec_from_file_location(
    'rplidar_c1_behavior', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'behavior.py'))
behavior = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(behavior)

# GET_LIDAR_CONF answers recorded from a real C1, after the 7-byte answer header:
# (configuration type, mode or None) -> data
RECORDED = {
    (0x70, None): '70000000 0200',
    (0x7C, None): '7c000000 0100',
    (0x01, None): '01000000 58022c01',
    (0x02, None): '02000000 03000000',
    (0x60, None): '60000000 00280000',
    (0x7F, 0): '7f000000 5374616e6461726400',
    (0x75, 0): '75000000 81',
    (0x71, 0): '71000000 00c80000',
    (0x74, 0): '74000000 00100000',
    (0x7F, 1): '7f000000 44656e7365426f6f737400',
    (0x75, 1): '75000000 85',
    (0x71, 1): '71000000 00c80000',
    (0x74, 1): '74000000 00280000',
    (0x7F, 2): '7f000000',
    (0x75, 3): '75000000',
    (0x76, None): '76000000',
    (0x81, None): '81000000',
}


def test_configuration_answers_match_a_real_c1():
    for (kind, mode), data in RECORDED.items():
        payload = struct.pack('<I', kind) + (struct.pack('<H', mode) if mode is not None else b'')
        assert behavior.RplidarC1.conf(payload) == bytes.fromhex(data.replace(' ', '')), \
            f'{kind:#x} mode {mode}'


def test_driver_padding_is_ignored():
    # Slamtec's driver pads configuration requests with zeros to 36 bytes.
    padded = struct.pack('<I', 0x7C) + bytes(32)
    assert behavior.RplidarC1.conf(padded) == bytes.fromhex('7c0000000100')
