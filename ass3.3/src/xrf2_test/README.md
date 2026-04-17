Package xrf2_test
-----------------------------------------------
### Description 
This package tests the loopcontroller by sending setpoints to drive forward and rotate 90 degrees.

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
* `sequence_step()`: Sends the next setpoints to XRF2.