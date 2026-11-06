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

"""Serve a device's receive service."""

from rclpy.qos import QoSProfile, ReliabilityPolicy
from simport_msgs.srv import Exchange

# A driver may send many packets at once (a motor bus: one per motor per cycle); the default
# service queue of 10 would drop some of them. simport's client uses the same depth.
EXCHANGE_QOS = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)


def serve(node, name, callback):
    """
    Serve the simport_msgs/srv/Exchange service `name` of a device's `receive`.

    `callback(request, response)` gets the decoded packet in `request.request` and appends
    the packets to send back to `response.replies`.
    """
    return node.create_service(Exchange, name, callback, qos_profile=EXCHANGE_QOS)
