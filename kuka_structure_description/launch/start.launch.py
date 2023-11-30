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
                     "kuka_structure_rviz.xacro"]
                ),
                " ",
                "prefix:=",
                prefix.perform(context),
                " ",
                "use_fake_hardware:=",
                use_fake_hardware.perform(context),]
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
         "ros2_controllers_config.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kuka_structure_description"),
         "config",
         "view_cell.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz_gui),
    )

    delta_utils_node = Node(
        name="delta_utils",
        package="ethercat_utils",
        executable="cia402_slave_manager",
        parameters=[ethercat_utils_config]
    )

    battery_cell_utils_node = Node(
        name="battery_cell_utils",
        package="ethercat_utils",
        executable="battery_cell_utils_manager",
        parameters=[ethercat_utils_config],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   "/controller_manager"],
    )
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller",
                   "-c",
                   "/controller_manager"],
    )
    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller",
                   "-c",
                   "/controller_manager"],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller",
                   "-c",
                   "/controller_manager",
                   "--stopped"], # start the controller in a stopped state
    )
    delta_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["delta_controller",
                   "-c",
                   "/controller_manager"],
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
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        delta_controller_spawner,
        digital_io_controller_spawner,
        ft_ati_controller_spawner,
        delta_utils_node,
        battery_cell_utils_node
        ]

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'prefix',
        default_value=''
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'rviz_gui',
        default_value="true"
    ))
    ld = LaunchDescription(launch_arguments +
                           [OpaqueFunction(function=launch_setup)])

    moveit_config = MoveItConfigsBuilder("kuka_structure",
                                         package_name="kuka_structure_moveit_config"
                                         ).to_moveit_configs()

    ld.add_action(IncludeLaunchDescription(
                      PythonLaunchDescriptionSource(
                          str(moveit_config.package_path / "launch/move_group.launch.py"))))

    return ld
