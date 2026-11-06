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
Run an example device: ros2 launch simport_examples example.launch.py example:=<name>.

Each example is a folder in devices/: every device file (*.yaml) in it is started, and so is
its behavior.py, if it has one (the node that answers the driver). Behaviours that run inside
the device are declared in the device file itself (`behavior:`). For your own device file, use
simport_ros's device.launch.py config_file:=<path>.
"""

import glob
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

DEVICES = os.path.join(get_package_share_directory('simport_examples'), 'devices')


def examples():
    """Names of the device folders."""
    return sorted(d for d in os.listdir(DEVICES) if os.path.isdir(os.path.join(DEVICES, d)))


def launch_example(context):
    """Turn the chosen device folder into processes."""
    example = LaunchConfiguration('example').perform(context)
    folder = os.path.join(DEVICES, example)
    if not os.path.isdir(folder):
        raise RuntimeError(f"unknown example '{example}'; examples: {', '.join(examples())}")
    sim_time = LaunchConfiguration('use_sim_time').perform(context)
    actions = []
    for path in sorted(glob.glob(os.path.join(folder, '*.yaml'))):
        with open(path) as f:
            name = yaml.safe_load(f)['device']
        actions.append(Node(
            package='simport_ros', executable='simport_device', name=name, output='screen',
            parameters=[{'config_file': path, 'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}]))
    behavior = os.path.join(folder, 'behavior.py')
    if os.path.exists(behavior):
        actions.append(ExecuteProcess(
            name=f'{example}_behavior', output='screen',
            cmd=[sys.executable, behavior, '--ros-args', '-r', f'__node:={example}_behavior',
                 '-p', f'use_sim_time:={sim_time}']))
    return actions


def generate_launch_description():
    """Declare the arguments; the processes depend on the chosen example."""
    return LaunchDescription([
        DeclareLaunchArgument('example', description='One of: ' + ', '.join(examples())),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_example),
    ])
