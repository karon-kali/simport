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

"""Protocol logic of the Dynamixel behaviour, without a ROS graph."""

import importlib.util
import os

_spec = importlib.util.spec_from_file_location(
    'dynamixel_behavior', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'behavior.py'))
behavior = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(behavior)
crc16, GOAL_POSITION, INDIRECT = behavior.crc16, behavior.GOAL_POSITION, behavior.INDIRECT
MODEL_NUMBER, Motor, read_only = behavior.MODEL_NUMBER, behavior.Motor, behavior.read_only


def test_dynamixel_crc_matches_the_protocol_documentation():
    # ROBOTIS e-manual, Protocol 2.0: ping ID 1 is FF FF FD 00 01 03 00 01 19 4E.
    assert crc16(bytes([0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01])) == 0x4E19


def test_dynamixel_control_table_and_indirect_addresses():
    m = Motor(11, 'pan', 1020, 1.0)
    assert m.get(MODEL_NUMBER, 2) == 1020
    assert m.get(GOAL_POSITION, 4) == 0
    m.put(GOAL_POSITION, 4, -1)
    assert m.get(GOAL_POSITION, 4, signed=True) == -1
    # Indirect Data byte k stands for the address stored in Indirect Address entry k.
    address, data, _ = INDIRECT[0]
    assert m.resolve(data) == data            # points at itself until configured
    m.put(address, 2, GOAL_POSITION)
    assert m.resolve(data) == GOAL_POSITION
    assert read_only(MODEL_NUMBER) and not read_only(GOAL_POSITION)
