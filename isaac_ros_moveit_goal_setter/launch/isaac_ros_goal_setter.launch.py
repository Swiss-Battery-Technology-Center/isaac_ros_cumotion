# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES',
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0


import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue



def get_robot_description():
    robot_ip = LaunchConfiguration('robot_ip')
    arm = LaunchConfiguration('arm', default='gen3')
    dof = LaunchConfiguration('dof', default='7')
    gripper = LaunchConfiguration('gripper', default='robotiq_2f_85')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare('kortex_description'), 'robots', 'gen3.xacro']),
            ' ', 'robot_ip:=', robot_ip,
            ' ', 'arm:=', arm,
            ' ', 'dof:=', dof,
            ' ', 'gripper:=', gripper,
            ' ', 'use_sim_time:=', use_sim_time,
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    return robot_description


def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare('kinova_gen3_7dof_robotiq_2f_85_moveit_config'), 'config', 'gen3.srdf']),
            ' ', 'name:=gen3', ' ', 'prefix:=""',
        ]
    )
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }
    return robot_description_semantic


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'robot_ip',
            description='IP address of the robot',
            default_value='192.168.1.10',
        ),
        DeclareLaunchArgument(
            'arm',
            default_value='gen3',
            description='Type of Kinova arm',
        ),
        DeclareLaunchArgument(
            'dof',
            default_value='7',
            description='Number of degrees of freedom',
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='robotiq_2f_85',
            description='Type of gripper',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),



    ]
    moveit_kinematics_params = PathJoinSubstitution(
        [FindPackageShare('kinova_gen3_7dof_robotiq_2f_85_moveit_config'), 'config', 'kinematics_cumotion.yaml']
    )
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    isaac_ros_moveit_goal_setter = Node(
        package='isaac_ros_moveit_goal_setter',
        executable='isaac_ros_moveit_goal_setter',
        name='isaac_ros_moveit_goal_setter',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_kinematics_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return launch.LaunchDescription(launch_args + [isaac_ros_moveit_goal_setter])
