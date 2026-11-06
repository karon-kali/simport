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

"""Run a node until Ctrl-C, SIGTERM or a ROS shutdown, and exit cleanly."""

import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def run_node(make_node, args=None):
    """
    Create a node with `make_node()` and spin it; return 0 when it was stopped.

    Ctrl-C and SIGTERM stop it cleanly. The signal handler only sets a flag: raising from it
    could interrupt rclpy in the middle of taking a message. The executor spins in short
    slices so that the flag is seen promptly.
    """
    stop = threading.Event()
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda signum, frame: stop.set())
    signal.signal(signal.SIGTERM, lambda signum, frame: stop.set())
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = make_node()
        executor.add_node(node)
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0
