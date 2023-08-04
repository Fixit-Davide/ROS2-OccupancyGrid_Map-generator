import os
import yaml
import pathlib
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='map_printer',
            executable='map_printer',
            name='map_printer',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
        )
])