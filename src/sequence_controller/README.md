Package sequence_controller
-----------------------------------------------
### Description 
This package implements a sequence controller that can either use a predefined list of setpoints or track a position.

### Inputs
`/input/object_position`  
        Type: geometry_msgs/msg/Point
        Only used when track_object is set to true.  

`/input/camera_position`  
        Type: geometry_msgs/msg/PointStamped
        Only used when track_object is set to true.

### Outputs
`/output/left_motor/setpoint_vel`  
        Type: example_interfaces/msg/Float64

`/output/right_motor/setpoint_vel`  
        Type: example_interfaces/msg/Float64

### Run
In a terminal run the following commands:
`ros2 run cam2image_vm2ros cam2image --ros-args --params-file src/cam2image_vm2ros/config/cam2image.yaml`  
`ros2 run object_position object_position`

### Parameters
bool `track_object` : Sets if the controller should track an object or use the setpoint lists.
vector<double> `left_waypoints: A list of velocity waypoints for the left motor
vector<double> `right_waypoints: A list of velocity waypoints for the right motor


### Core components 
* `sequence_step()`: Publishes the next setpoint from the setpoint lists, called by a timer.
* `objectPosition_callback()`: Updates the object position and calls calc_setpoint().
* `cameraPosition_callback()`: Updates the camera position and calls calc_setpoint().
* `calc_setpoint()`: Calculate the error and use that to calculate and publish a setpoint.