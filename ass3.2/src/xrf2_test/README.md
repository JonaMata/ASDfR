Package xrf2_test
-----------------------------------------------
### Description 
This package tests the Ros-Xenomai communication by sending a sequence of setpoints and printing any received messages

### Inputs
`/XenoState`  
        Type: std_msgs/msg/Int32

`/Xeno2Ros`
        Type: xrf2_msgs/msg/Xeno2Ros

### Outputs
`/XenoCmd`  
        Type: std_msgs/msg/Int32

`/Ros2Xeno`
        Type: xrf2_msgs/msg/Ros2Xeno

### Run
Run the command:
`ros2 run xrf2_test xrf2_test`


### Core components 
* `sequence_step()`: Sends the next setpoints from the setpoint list to XRF2.