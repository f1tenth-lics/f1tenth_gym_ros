from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racecar_description'), 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'sim': 'true',
            'namespace': 'ego_racecar',
            'use_namespace': 'false'
        }.items()
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racecar_control'), 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'namespace': 'ego_racecar',
            'use_namespace': 'false'
        }.items()
    )

    nodes = [
        description_launch,
        control_launch,
    ]

    return LaunchDescription(nodes)