# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'maps', 'levine.yaml'),
            description='Full path to map file to load'
        ),
    ]

    map = LaunchConfiguration('map')

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, {'map_path': map}],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    )
    localization_params = os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'config', 'localization_params.yaml')
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racecar_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'params_file': localization_params,
            'namespace': config_dict['bridge']['ros__parameters']['ego_namespace'],
            'use_namespace': 'true',
            'map': map,
            'autostart': 'true',
        }.items()
    )
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     parameters=[{'yaml_filename': map_path},
    #                 {'topic': 'map'},
    #                 {'frame_id': 'map'},
    #                 {'output': 'screen'},
    #                 {'use_sim_time': True}]
    # )
    # nav_lifecycle_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'node_names': ['map_server']}]
    # )
    # ego_robot_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='ego_robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
    #     remappings=[('/robot_description', 'ego_robot_description')]
    # )
    ego_robot_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racecar_description'), 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'sim': 'true',
            'namespace': config_dict['bridge']['ros__parameters']['ego_namespace'],
            'use_namespace': 'true'
        }.items()
    )
    ego_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racecar_control'), 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'namespace': config_dict['bridge']['ros__parameters']['ego_namespace'],
            'use_namespace': 'true'
        }.items()
    )

    # opp_robot_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='opp_robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
    #     remappings=[('/robot_description', 'opp_robot_description')]
    # )
    opp_robot_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racecar_description'), 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'sim': 'true',
            'namespace': config_dict['bridge']['ros__parameters']['opp_namespace'],
            'use_namespace': 'true'
        }.items()
    )
    opp_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racecar_control'), 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'namespace': config_dict['bridge']['ros__parameters']['opp_namespace'],
            'use_namespace': 'true'
        }.items()
    )


    # finalize
    ld.add_action(*declared_arguments)
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(localization_launch)
    # ld.add_action(nav_lifecycle_node)
    # ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(ego_controller)
    if has_opp:
        ld.add_action(opp_robot_publisher)
        ld.add_action(opp_controller)

    return ld