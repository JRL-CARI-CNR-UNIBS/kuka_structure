# Copyright 2023 Áron Svastits
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
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):

    # args that can be set from the command line or a default will be used
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    rviz_gui = LaunchConfiguration('rviz_gui')

    robot_description = {
        'robot_description':
        ParameterValue(
           Command(
               [PathJoinSubstitution([FindExecutable(name='xacro')]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare('kuka_structure_description'),
                     "urdf",
                     "only_gpio.xacro"]
                ),]
           ),
           value_type=str
        )
    }
    ethercat_utils_config = PathJoinSubstitution(
                                [FindPackageShare('kuka_structure_description'),
                                 "config",
                                 "ethercat_utils_config.yaml"]
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("kuka_structure_description"),
         "config",
         "only_gpio_ros2_controllers_config.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )
    battery_cell_utils_node = Node(
        name="battery_cell_utils",
        package="ethercat_utils",
        executable="battery_cell_utils_manager",
        parameters=[ethercat_utils_config],
        output="screen"
    )
    digital_io_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["digital_io_controller",
                   "-c",
                   "/controller_manager"],
    )
    ft_ati_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ft_ati_controller",
                   "-c",
                   "/controller_manager"],
    )

    nodes_to_start = [
        control_node,
        digital_io_controller_spawner,
        ft_ati_controller_spawner,
        battery_cell_utils_node
        ]

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    ld = LaunchDescription(launch_arguments +
                           [OpaqueFunction(function=launch_setup)])

    return ld
