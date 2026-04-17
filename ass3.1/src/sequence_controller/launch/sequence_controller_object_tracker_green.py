from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    cam2image = Node(
        package='cam2image_vm2ros',
        executable='cam2image',
        name='cam2image',
        parameters=[PathJoinSubstitution([
            FindPackageShare('sequence_controller'), 'config', 'cam2image_relbot.yaml'])
        ],)

    object_position = Node(
        package='object_position',
        executable='object_position',
        name='object_position',
        remappings=[
            ('/output/object_position', '/input/object_position'),
        ],
        parameters=[{
            'from_center': True,
            'threshold': 10,
        }]
    )

    sequence_controller = Node(
        package='sequence_controller',
        executable='sequence_controller',
        name='sequence_controller',
        remappings=[
            ('/output/left_motor/setpoint_vel', '/input/left_motor/setpoint_vel'),
            ('/output/right_motor/setpoint_vel', '/input/right_motor/setpoint_vel'),
        ],
        parameters=[
            {
                'tau': 0.1,
            }
        ],
    )

    return LaunchDescription([
        cam2image,
        object_position,
        sequence_controller,
    ])