from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    sequence_controller = Node(
        package='sequence_controller',
        executable='sequence_controller',
        name='sequence_controller',
        remappings=[
            ('/output/left_motor/setpoint_vel', '/input/left_motor/setpoint_vel'),
            ('/output/right_motor/setpoint_vel', '/input/right_motor/setpoint_vel'),
        ],
        parameters=[{
            'left_waypoints': [0.0, 1.0, 0.0, -1.0],
            'right_waypoints': [0.0, 1.0, 0.0, -1.0],
        }]
    )

    relbot_adapter = Node(
        package='relbot_adapter',
        executable='relbot_adapter',
        name='relbot_adapter',
        remappings=[
            ('/output/motor_cmd', '/input/motor_cmd'),
            ('/input/twist', '/keyboard/twist'),
        ],
        parameters=[{
            'track_object': False,
            'tau': 10.0,
        }],
    )

    relbot_simulator = Node(
        package='relbot_simulator',
        executable='relbot_simulator',
        name='relbot_simulator',
    )

    return LaunchDescription([
        sequence_controller,
        relbot_adapter,
        relbot_simulator,
    ])