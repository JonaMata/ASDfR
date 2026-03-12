from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    seq13 = Node(
        package='roundtrip_test',
        executable='seq13',
        name='seq13',
    )

    loop13 = Node(
        package='roundtrip_test',
        executable='loop13',
        name='loop13',
    )

    return LaunchDescription([
        seq13,
        loop13,
    ])