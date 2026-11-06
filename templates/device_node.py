#!/usr/bin/env python3
"""
Template: the node that decides what your device does.

simport turns the driver's packets into calls of a simport_msgs/srv/Exchange service (a
`receive` with `to: { service: ... }`). The request holds the decoded packet; each record you
put in the response is encoded with one of the receive's `reply` messages and sent back to
the driver. Answer with nothing and the device stays silent, like a real one ignoring a
command.

Copy it next to your device file and edit `on_command`; the behavior.py files in
simport_examples/devices/ are complete examples for real devices. As it is, it serves templates/device.yaml:

    python3 device_node.py
"""

import sys

from rclpy.node import Node
from simport import fields_of, make_record, run_node, serve

PING = 0x01


class MyDevice(Node):
    """Answers the commands of /my_sensor/command."""

    def __init__(self):
        super().__init__('my_sensor_node')
        serve(self, '/my_sensor/command', self.on_command)

    def on_command(self, request, response):
        # The decoded packet, by field name: {'type': 2, 'cmd': 1, 'arg': 0, ...}. The message
        # name (request.request.name) says which receive it came from.
        command = fields_of(request.request)

        if command['cmd'] == PING:
            # make_record(<send message name>, field=value, ...): constants, counters,
            # lengths and checksums are filled in by simport. With a single `reply`
            # message the name may be left out.
            response.replies.append(make_record('ack', cmd=command['cmd'], status=0))
        else:
            response.replies.append(make_record('ack', cmd=command['cmd'], status=1))

        # Typical things a device node also does:
        # - publish on the topic a `send` reads from, to start or stop a stream
        # - keep registers / state between calls
        # - answer with several packets (append more records)
        return response


def main():
    return run_node(MyDevice)


if __name__ == '__main__':
    sys.exit(main())
