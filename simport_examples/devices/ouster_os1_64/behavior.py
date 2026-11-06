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
HTTP API of an Ouster OS1-64, firmware 3.2, for http.yaml next to this file.

Serves /<device>/http, where simport sends the driver's HTTP requests, with the sensor's
metadata and configuration from the JSON files in `data_dir` (default: data/ next to this
data/ouster_os1_64). Configuration changes (including udp_dest "@auto" and reinitialize) are
acknowledged; the sensor keeps its settings.

    python3 behavior.py [--ros-args -p device:=os1]
"""

import os
import sys

from rclpy.node import Node
from simport import fields_of, make_record, run_node, serve


FIRMWARE = '{"fw": "ousteros-image-prod-aries-v3.2.0+20240101000000"}'


class OusterOs1(Node):
    """Serves /<device>/http."""

    def __init__(self):
        super().__init__('ouster_os1_64')
        device = self.declare_parameter('device', 'os1').value
        data_dir = self.declare_parameter('data_dir', '').value or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'data')

        def load(name):
            with open(os.path.join(data_dir, name)) as f:
                return f.read()

        self.routes = {
            ('GET', '/api/v1/sensor/metadata/sensor_info'): load('sensor_info.json'),
            ('GET', '/api/v1/sensor/metadata'): load('metadata.json'),
            ('GET', '/api/v1/sensor/config'): load('config.json'),  # active and staged alike
            ('POST', '/api/v1/sensor/config'): '{}',
            ('GET', '/api/v1/system/firmware'): FIRMWARE,
            ('GET', '/api/v1/user/data'): '""',
        }
        serve(self, f'/{device}/http', self.on_request)

    def on_request(self, request, response):
        req = fields_of(request.request)
        body = self.routes.get((req['method'], req['path']))
        if body is None:
            response.replies.append(make_record(status=404, body='{"error": "not found"}'))
        else:
            response.replies.append(make_record(status=200, body=body))
        return response


def main():
    """Run the node."""
    return run_node(OusterOs1)


if __name__ == '__main__':
    sys.exit(main())
