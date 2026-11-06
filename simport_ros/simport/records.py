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
Read and write simport_msgs/Record messages as plain Python values.

A device's `receive` messages arrive as Records (on a topic, or as the request of a
simport_msgs/srv/Exchange service); the service answers with Records that simport encodes
with the receive's `reply` send messages::

    from simport import fields_of, make_record

    def on_command(request, response):
        cmd = fields_of(request.request)        # {'cmd': 1, 'arg': 42, ...}
        response.replies.append(make_record('ack', cmd=cmd['cmd'], arg=cmd['arg']))
        return response
"""

from simport_msgs.msg import Record, RecordField


def _value(field):
    kind = field.kind
    if kind == RecordField.KIND_BOOL:
        return field.number != 0.0
    if kind in (RecordField.KIND_INT, RecordField.KIND_UINT):
        return int(field.number)
    if kind == RecordField.KIND_FLOAT:
        return field.number
    if kind == RecordField.KIND_STRING:
        return field.text
    if kind == RecordField.KIND_BYTES:
        return bytes(field.bytes)
    return list(field.array)


def fields_of(record):
    """Return the fields of a Record as a dict of Python values (int, float, bytes, ...)."""
    return {f.name: _value(f) for f in record.fields}


def make_record(name='', **fields):
    """
    Build a Record from keyword arguments.

    `name` picks the send message the record is encoded as, when a receive lists several
    replies; leave it empty for the first. Values may be bool, int, float, str, bytes (or a
    bytearray) and lists of numbers; simport converts them to each field's type. Fields left
    out keep their defaults (constants, counters and checksums are filled in by simport).
    """
    msg = Record()
    msg.name = name
    for key, value in fields.items():
        f = RecordField()
        f.name = key
        if isinstance(value, bool):
            f.kind = RecordField.KIND_BOOL
            f.number = 1.0 if value else 0.0
        elif isinstance(value, int):
            f.kind = RecordField.KIND_INT if value < 0 else RecordField.KIND_UINT
            f.number = float(value)
        elif isinstance(value, float):
            f.kind = RecordField.KIND_FLOAT
            f.number = value
        elif isinstance(value, str):
            f.kind = RecordField.KIND_STRING
            f.text = value
        elif isinstance(value, (bytes, bytearray)):
            f.kind = RecordField.KIND_BYTES
            f.bytes = bytes(value)
        else:
            f.kind = RecordField.KIND_FLOAT_ARRAY
            f.array = [float(v) for v in value]
        f.text = f.text or str(value)
        msg.fields.append(f)
    return msg
