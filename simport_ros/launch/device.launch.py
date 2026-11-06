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

"""Launch one simport device from a device YAML file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare arguments and the device node."""
    args = [
        DeclareLaunchArgument('config_file', description='Path to the device YAML file'),
        DeclareLaunchArgument('name', default_value='simport_device', description='Node name'),
        DeclareLaunchArgument('namespace', default_value='', description='Node namespace'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate the device on startup'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time from /clock'),
        DeclareLaunchArgument('log_level', default_value='info', description='Logging level'),
    ]
    node = Node(
        package='simport_ros',
        executable='simport_device',
        name=LaunchConfiguration('name'),
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    return LaunchDescription(args + [node])
