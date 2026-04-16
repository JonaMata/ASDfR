Package sequence_controller
-----------------------------------------------
### Description 
This package implements a sequence controller that can create setpoints from a tracked object position

### Inputs
`/input/object_position`  
        Type: geometry_msgs/msg/Point 

`/XenoState`  
        Type: std_msgs/msg/Int32

`/Xeno2Ros`
        Type: xrf2_msgs/msg/Xeno2Ros

### Outputs
`/XenoCmd`  
        Type: std_msgs/msg/Int32

`/Ros2Xeno`
        Type: xrf2_msgs/msg/Ros2Xeno


### Parameters
double `tau` : The scaling factor of the rotational steer setpoints.
double `miauw` : The scaling factor of the forward and backward steer setpoints.
double `want_ball_size`: The ball size or distance to the ball the RELbot should try to maintain.


### Core components 
* `topic_callback()`: Calculate new setpoints from a received object position message.