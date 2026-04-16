from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

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
	    'show_mask': False,
        }]
    )

    object_tracker_simple = Node(
        package='sequence_controller',
        executable='sequence_controller',
        name='sequence_controller',
        parameters=[
            {
                'track_object': True,
                'tau': 0.03,
		'miauw': 0.3,
            }
        ],
    )

    # relbot_adapter = Node(
    #     package='relbot_adapter',
    #     executable='relbot_adapter',
    #     name='relbot_adapter',
    #     remappings=[
    #         ('/output/motor_cmd', '/input/motor_cmd'),
    #         ('/input/twist', '/keyboard/twist'),
    #     ]
    # )

    # relbot_simulator = Node(
    #     package='relbot_simulator',
    #     executable='relbot_simulator',
    #     name='relbot_simulator',
    # )

    return LaunchDescription([
        object_position,
        object_tracker_simple,
        # relbot_adapter,
        # relbot_simulator,
    ])
