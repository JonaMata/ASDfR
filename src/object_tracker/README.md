Package object_tracker
-----------------------------------------------
### Description 
This package implements an object_tracker that uses the moving camera feed of the RELbot to publish setpoints for the motors to steer the RELbot to look at the object.

### Inputs
`/input/object_position`  
        Type: geometry_msgs/msg/Point 

### Outputs
`/output/left_motor/setpoint_vel`  
        Type: example_interfaces/msg/Float64

`/output/right_motor/setpoint_vel`  
        Type: example_interfaces/msg/Float64

### Run
In a terminal run the following commands:

`ros2 run cam2image_vm2ros cam2image --ros-args --params-file src/cam2image_vm2ros/config/cam2image.yaml`  
`ros2 launch object_tracker object_tracker_launch.py`

### Parameters
double `tau`: A factor tau to scale the error when creating the velocity setpoint


### Core components 
* `topic_callback()`: Calculates a publishes velocity setpoints from the error received from the object_position.