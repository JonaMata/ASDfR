from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cam2image = Node(
        package='cam2image_vm2ros',
        executable='cam2image',
        name='cam2image',
        parameters="",
    )

    sequence_controller = Node(
        package='sequence_controller',
        executable='sequence_controller',
        name='sequence_controller',
        remappings=[
            {'/output/left_motor/setpoint_vel', '/input/left_motor/setpoint_vel'},
            {'/output/right_motor/setpoint_vel', '/input/right_motor/setpoint_vel'},
        ]
    )

    relbot_adapter = Node(
        package='relbot_adapter',
        executable='relbot_adapter',
        name='relbot_adapter',
        remappings=[
            {'/output/motor_cmd', '/input/motor_cmd'},
            {'/input/twist', '/keyboard/twist'},
        ]
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