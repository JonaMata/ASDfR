from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('sequence_controller'),
        'config',
        'config_simple.yaml'
    )

    cam2image = Node(
        package='cam2image_vm2ros',
        executable='cam2image',
        name='cam2image',
        parameters=[config],
    )

    object_position = Node(
        package='image_analysis',
        executable='object_position',
        name='object_position',
        remappings=[
            ('/output/object_position', '/input/object_position'),
        ]
    )

    object_tracker_simple = Node(
        package='sequence_controller',
        executable='object_tracker_simple',
        name='object_tracker_simple',
        remappings=[
            ('/output/left_motor/setpoint_vel', '/input/left_motor/setpoint_vel'),
            ('/output/right_motor/setpoint_vel', '/input/right_motor/setpoint_vel'),
        ],
        parameters=[config],
    )

    relbot_adapter = Node(
        package='relbot_adapter',
        executable='relbot_adapter',
        name='relbot_adapter',
        remappings=[
            ('/output/motor_cmd', '/input/motor_cmd'),
            ('/input/twist', '/keyboard/twist'),
        ]
    )

    relbot_simulator = Node(
        package='relbot_simulator',
        executable='relbot_simulator',
        name='relbot_simulator',
        remappings=[
            ('/output/camera_position', '/input/camera_position'),
        ],
    )

    return LaunchDescription([
        cam2image,
        object_position,
        object_tracker_simple,
        relbot_adapter,
        relbot_simulator,
    ])