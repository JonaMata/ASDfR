Package sequence_controller
-----------------------------------------------
### Description 
This package implements a sequence controller that will publish setpoints .

### Inputs
`/input/object_position`  
        Type: geometry_msgs/msg/Point

### Outputs
`/output/left_motor/setpoint_vel`  
        Type: example_interfaces/msg/Float64

`/output/right_motor/setpoint_vel`  
        Type: example_interfaces/msg/Float64

### Run
Run the following commands:

`ros2 run cam2image_vm2ros cam2image --ros-args --params-file src/sequence_controller/config/cam2image_relbot.yaml`  
`ros2 launch sequence_controller object_tracker_simple_launch.py`

### Parameters
double `tau` : The scaling factor of the rotational steer setpoints.
double `want_ball_size`: The ball size or distance to the ball the RELbot should try to maintain.

### Core components 
* `topic_callback()`: Receive the object position, calculate the error and set the new setpoint.