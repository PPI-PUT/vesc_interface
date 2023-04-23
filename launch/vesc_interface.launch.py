# Copyright 2023 Perception for Physical Interaction Laboratory at Poznan University of Technology
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare(LaunchConfiguration('param_file_pkg'))
    vesc_driver_prefix = FindPackageShare('vesc_driver')
    config = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('param_file')])
    vesc_driver_config = PathJoinSubstitution([vesc_driver_prefix, 'params', 'vesc_config.yaml'])

    container = ComposableNodeContainer(
        name='vesc_interface_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='vesc_interface',
                    # remappings=[("/sensors/imu", "/imu")],
                    plugin='vesc_interface::VescInterfaceNode',
                    name='vesc_interface_node',
                    parameters=[
                        config,
                        vesc_driver_config
                    ],
                ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    return [
        container
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'param_file_pkg',
            default_value='vesc_interface',
            description='Package name which contains param file.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'param_file',
            default_value='param/defaults.param.yaml',
            description='Param file (relative path).'
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
