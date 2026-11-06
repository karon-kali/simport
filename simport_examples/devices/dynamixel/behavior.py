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
Dynamixel X-series motors, Protocol 2.0, for dynamixel.yaml next to this file.

Serves /<device>/instruction, where simport sends every instruction packet: keeps each
motor's control table, answers ping, read, write, reg write/action, reboot, factory reset,
sync/bulk read and write and their fast variants, and moves the simulated joints.
Goal writes become std_msgs/Float64 commands on `command_topic` (default
/sim/<joint>/cmd_pos, or cmd_vel / cmd_eff in velocity / current mode); present values come
from `joint_states_topic` (default /sim/joint_states).

    python3 behavior.py [--ros-args -p device:=dynamixel -p ids:=[11,12] \
      -p joints:=[dxl_pan,dxl_tilt]]
"""

import sys
import time

from rclpy.node import Node
from sensor_msgs.msg import JointState
from simport import fields_of, make_record, run_node, serve
from std_msgs.msg import Float64


# Instructions.
PING, READ, WRITE, REG_WRITE, ACTION, FACTORY_RESET, REBOOT, CLEAR = (
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x10)
SYNC_READ, SYNC_WRITE, FAST_SYNC_READ = 0x82, 0x83, 0x8A
BULK_READ, BULK_WRITE, FAST_BULK_READ = 0x92, 0x93, 0x9A
BROADCAST = 0xFE

# Status errors.
ERR_INSTRUCTION, ERR_DATA_RANGE, ERR_DATA_LENGTH, ERR_ACCESS = 0x02, 0x04, 0x05, 0x07

# X-series control table: address -> (size, power-up value).
MODEL_NUMBER, FIRMWARE, ID, DRIVE_MODE, OPERATING_MODE = 0, 6, 7, 10, 11
HOMING_OFFSET, MOVING_THRESHOLD = 20, 24
TORQUE_ENABLE, STATUS_RETURN_LEVEL, REGISTERED, HARDWARE_ERROR = 64, 68, 69, 70
GOAL_CURRENT, GOAL_VELOCITY, GOAL_POSITION = 102, 104, 116
REALTIME_TICK, MOVING, MOVING_STATUS = 120, 122, 123
PRESENT_PWM, PRESENT_CURRENT, PRESENT_VELOCITY, PRESENT_POSITION = 124, 126, 128, 132
VELOCITY_TRAJECTORY, POSITION_TRAJECTORY = 136, 140
RAM_START, TABLE_SIZE = 64, 1024
DEFAULTS = {
    FIRMWARE: (1, 52),  # fast sync/bulk read capable
    8: (1, 3),  # baud rate: 1 Mbps
    9: (1, 0),  # return delay
    OPERATING_MODE: (1, 3),  # position control
    13: (1, 2),  # protocol 2.0
    MOVING_THRESHOLD: (4, 10),
    31: (1, 80), 32: (2, 160), 34: (2, 95),  # temperature and voltage limits
    36: (2, 885), 38: (2, 1193), 44: (4, 200),  # PWM, current, velocity limits
    48: (4, 4095), 52: (4, 0),  # position limits
    63: (1, 52),  # shutdown
    STATUS_RETURN_LEVEL: (1, 2),
    76: (2, 1920), 78: (2, 100), 84: (2, 800),  # gains
    100: (2, 885),  # goal PWM
    144: (2, 120), 146: (1, 35),  # present voltage and temperature
}
# Indirect address blocks: (first Indirect Address, first Indirect Data, entries).
INDIRECT = ((168, 224, 28), (578, 634, 28))

POSITION_UNIT = 0.0015339807878856412  # rad per tick (4096 per turn)
VELOCITY_UNIT = 0.0239691227           # rad/s per 0.229 rpm
CURRENT_UNIT = 0.00269                 # A per 2.69 mA


def crc16(data):
    """CRC-16/BUYPASS, as used by Protocol 2.0."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def le16(data, at):
    return data[at] | data[at + 1] << 8


class Motor:
    """One motor: its control table and the joint it drives."""

    def __init__(self, motor_id, joint, model, torque_constant):
        self.id, self.joint, self.model = motor_id, joint, model
        self.torque_constant = torque_constant
        self.reset()

    def reset(self):
        self.table = bytearray(TABLE_SIZE)
        self.put(MODEL_NUMBER, 2, self.model)
        self.put(ID, 1, self.id)
        for addr, (size, value) in DEFAULTS.items():
            self.put(addr, size, value)
        for address, data, count in INDIRECT:  # each points at its own data byte
            for k in range(count):
                self.put(address + 2 * k, 2, data + k)
        self.registered = None

    def get(self, addr, size, signed=False):
        return int.from_bytes(self.table[addr:addr + size], 'little', signed=signed)

    def put(self, addr, size, value):
        self.table[addr:addr + size] = (value & ((1 << 8 * size) - 1)).to_bytes(size, 'little')

    def sign(self):
        return -1.0 if self.table[DRIVE_MODE] & 1 else 1.0

    def resolve(self, addr):
        """Return the control-table address an Indirect Data byte stands for."""
        for address, data, count in INDIRECT:
            if data <= addr < data + count:
                target = self.get(address + 2 * (addr - data), 2)
                return target if target < TABLE_SIZE else addr
        return addr


def read_only(addr):
    return addr < ID or addr in (REGISTERED, HARDWARE_ERROR) or REALTIME_TICK <= addr < 168


def indirect_address(addr):
    return any(address <= addr < address + 2 * count for address, _, count in INDIRECT)


class DynamixelBus(Node):
    """Serves /<device>/instruction for the motors on the bus."""

    def __init__(self):
        super().__init__('dynamixel')
        device = self.declare_parameter('device', 'dynamixel').value
        ids = self.declare_parameter('ids', [11, 12]).value
        joints = self.declare_parameter('joints', ['dxl_pan', 'dxl_tilt']).value
        model = self.declare_parameter('model', 1020).value  # XM430-W350
        torque_constant = self.declare_parameter('torque_constant', 1.0).value  # Nm/A
        command_topic = self.declare_parameter('command_topic', '/sim/{joint}/cmd_{mode}').value
        states_topic = self.declare_parameter('joint_states_topic', '/sim/joint_states').value
        self.motors = [Motor(i, j, model, torque_constant) for i, j in zip(ids, joints)]
        self.started = time.monotonic()
        self.states = {}
        self.commands = {}
        for m in self.motors:
            for mode in ('pos', 'vel', 'eff'):
                self.commands[(m.joint, mode)] = self.create_publisher(
                    Float64, command_topic.format(joint=m.joint, mode=mode), 10)
        self.create_subscription(JointState, states_topic, self.on_joint_states, 10)
        serve(self, f'/{device}/instruction', self.on_instruction)

    def on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            self.states[name] = (
                msg.position[i] if i < len(msg.position) else 0.0,
                msg.velocity[i] if i < len(msg.velocity) else 0.0,
                msg.effort[i] if i < len(msg.effort) else 0.0)

    def by_id(self, motor_id):
        return next((m for m in self.motors if m.table[ID] == motor_id), None)

    # -- the control table ---------------------------------------------------------------------

    def refresh_present(self, m):
        position, velocity, effort = self.states.get(m.joint, (0.0, 0.0, 0.0))
        sign = m.sign()
        homing = m.get(HOMING_OFFSET, 4, signed=True)
        velocity_raw = round(sign * velocity / VELOCITY_UNIT)
        current = round(sign * effort / m.torque_constant / CURRENT_UNIT)
        m.put(PRESENT_POSITION, 4, round(sign * position / POSITION_UNIT) + 2048 + homing)
        m.put(PRESENT_VELOCITY, 4, velocity_raw)
        m.put(PRESENT_CURRENT, 2, max(-32768, min(32767, current)))
        m.put(POSITION_TRAJECTORY, 4, m.get(GOAL_POSITION, 4))
        m.put(VELOCITY_TRAJECTORY, 4, velocity_raw)
        m.put(PRESENT_PWM, 2, 0)
        moving = abs(velocity_raw) > m.get(MOVING_THRESHOLD, 4)
        m.table[MOVING] = 1 if moving else 0
        m.table[MOVING_STATUS] = 0x00 if moving else 0x01  # in position when settled
        m.put(REALTIME_TICK, 2, int((time.monotonic() - self.started) * 1000) % 32768)

    def read_bytes(self, m, addr, length):
        if length == 0 or addr + length > TABLE_SIZE:
            return ERR_DATA_LENGTH, b''
        self.refresh_present(m)
        return 0, bytes(m.table[m.resolve(addr + i)] for i in range(length))

    def write_bytes(self, m, addr, data):
        if not data or addr + len(data) > TABLE_SIZE:
            return ERR_DATA_LENGTH
        torque_was_on = m.table[TORQUE_ENABLE] != 0
        targets = [m.resolve(addr + i) for i in range(len(data))]
        for a in targets:
            if read_only(a):
                return ERR_ACCESS
            # EEPROM and indirect addresses are locked while torque is on.
            if torque_was_on and (a < RAM_START or indirect_address(a)):
                return ERR_ACCESS
        if OPERATING_MODE in targets:
            if data[targets.index(OPERATING_MODE)] not in (0, 1, 3, 4, 5, 16):
                return ERR_DATA_RANGE
        for a, value in zip(targets, data):
            m.table[a] = value
        self.apply_goals(m, min(targets), max(targets), torque_was_on)
        return 0

    def apply_goals(self, m, first, last, torque_was_on):
        def touched(addr, size):
            return first < addr + size and addr <= last

        torque_on = m.table[TORQUE_ENABLE] != 0
        if torque_on and not torque_was_on:
            # Torque on: the motor holds where it is; Goal Position follows Present Position.
            self.refresh_present(m)
            m.put(GOAL_POSITION, 4, m.get(PRESENT_POSITION, 4))
        if not torque_on:
            return
        mode, sign = m.table[OPERATING_MODE], m.sign()
        homing = m.get(HOMING_OFFSET, 4, signed=True)
        if mode in (3, 4, 5) and (touched(GOAL_POSITION, 4) or not torque_was_on):
            raw = m.get(GOAL_POSITION, 4, signed=True)
            self.command(m, 'pos', sign * (raw - 2048 - homing) * POSITION_UNIT)
        elif mode == 1 and (touched(GOAL_VELOCITY, 4) or not torque_was_on):
            self.command(m, 'vel', sign * m.get(GOAL_VELOCITY, 4, signed=True) * VELOCITY_UNIT)
        elif mode == 0 and (touched(GOAL_CURRENT, 2) or not torque_was_on):
            raw = m.get(GOAL_CURRENT, 2, signed=True)
            self.command(m, 'eff', sign * raw * CURRENT_UNIT * m.torque_constant)

    def command(self, m, mode, value):
        self.commands[(m.joint, mode)].publish(Float64(data=float(value)))

    # -- instructions --------------------------------------------------------------------------

    def on_instruction(self, request, response):
        packet = fields_of(request.request)
        motor_id, inst, p = packet['id'], packet['instruction'], packet['params']
        broadcast = motor_id == BROADCAST
        target = None if broadcast else self.by_id(motor_id)
        if not broadcast and target is None:
            return response  # another device on the bus
        addressed = self.motors if broadcast else [target]

        def status(m, error=0, params=b'', packet_id=None):
            response.replies.append(make_record(
                id=m.table[ID] if packet_id is None else packet_id, error=error, params=params))

        def replies_to(m, is_read):
            level = m.table[STATUS_RETURN_LEVEL]
            return level >= 2 or (level == 1 and is_read)

        if inst == PING:
            for m in addressed:
                model = m.get(MODEL_NUMBER, 2).to_bytes(2, 'little')
                status(m, 0, model + bytes([m.table[FIRMWARE]]))
        elif inst == READ:
            if broadcast:
                return response
            if len(p) != 4:
                status(target, ERR_DATA_LENGTH)
            else:
                error, data = self.read_bytes(target, le16(p, 0), le16(p, 2))
                if replies_to(target, True):
                    status(target, error, b'' if error else data)
        elif inst in (WRITE, REG_WRITE):
            if len(p) < 3:
                if not broadcast:
                    status(target, ERR_DATA_LENGTH)
                return response
            addr, data = le16(p, 0), p[2:]
            for m in addressed:
                error = 0
                if inst == WRITE:
                    error = self.write_bytes(m, addr, data)
                else:
                    m.registered = (addr, data)
                    m.table[REGISTERED] = 1
                if not broadcast and replies_to(m, False):
                    status(m, error)
        elif inst in (ACTION, REBOOT, CLEAR, FACTORY_RESET):
            for m in addressed:
                if inst == ACTION and m.registered:
                    self.write_bytes(m, *m.registered)
                    m.registered = None
                    m.table[REGISTERED] = 0
                elif inst == REBOOT:
                    m.table[TORQUE_ENABLE] = 0
                    m.table[HARDWARE_ERROR] = 0
                elif inst == FACTORY_RESET:
                    keep_id = m.table[ID]
                    m.reset()
                    m.table[ID] = keep_id if p and p[0] != 0xFF else 1
                if not broadcast and replies_to(m, False):
                    status(m)
        elif inst in (SYNC_READ, FAST_SYNC_READ, BULK_READ, FAST_BULK_READ):
            self.read_many(inst, p, status)
        elif inst == SYNC_WRITE and len(p) >= 4:
            addr, length = le16(p, 0), le16(p, 2)
            i = 4
            while length and i + 1 + length <= len(p):
                if m := self.by_id(p[i]):
                    self.write_bytes(m, addr, p[i + 1:i + 1 + length])
                i += 1 + length
        elif inst == BULK_WRITE:
            i = 0
            while i + 5 <= len(p):
                addr, length = le16(p, i + 1), le16(p, i + 3)
                if i + 5 + length > len(p):
                    break
                if m := self.by_id(p[i]):
                    self.write_bytes(m, addr, p[i + 5:i + 5 + length])
                i += 5 + length
        elif inst not in (SYNC_WRITE,) and not broadcast:
            status(target, ERR_INSTRUCTION)
        return response  # sync and bulk writes have no status packets

    def read_many(self, inst, p, status):
        """Sync read: address, length, ids...; bulk read: (id, address, length)..."""
        if inst in (SYNC_READ, FAST_SYNC_READ):
            if len(p) < 5:
                return
            reads = [(i, le16(p, 0), le16(p, 2)) for i in p[4:]]
        else:
            if len(p) < 5 or len(p) % 5:
                return
            reads = [(p[i], le16(p, i + 1), le16(p, i + 3)) for i in range(0, len(p), 5)]
        fast = inst in (FAST_SYNC_READ, FAST_BULK_READ)
        combined = bytearray([0x55])  # fast: one packet from all motors
        params = bytearray()
        first = None
        for motor_id, addr, length in reads:
            m = self.by_id(motor_id)
            if m is None:
                continue  # absent motors do not answer
            error, data = self.read_bytes(m, addr, length)
            if not fast:
                status(m, error, b'' if error else data)
                continue
            if first is None:
                first = error  # the first motor's error is the packet's error field
                block = bytes([motor_id]) + (data if not error else bytes(length))
                combined += bytes([error]) + block
                params += block
                continue
            # A CRC closes the previous motor's block (drivers do not check it; the packet CRC
            # protects the whole reply).
            crc = crc16(combined).to_bytes(2, 'little')
            block = bytes([error, motor_id]) + (data if not error else bytes(length))
            combined += crc + block
            params += crc + block
        if fast and first is not None:
            status(None, first, bytes(params), packet_id=BROADCAST)


def main():
    """Run the node."""
    return run_node(DynamixelBus)


if __name__ == '__main__':
    sys.exit(main())
