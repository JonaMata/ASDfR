Package object_position
-----------------------------------------------
### Description 
This package implements a node that finds the position of an object in an image. It checks each pixel against a brightness threshold to define whether it is part of the object and then finds the center of mass of all object pixels.

### Inputs
`/image`  
        Type: sensor_msgs/msg/Image  

### Outputs
`/output/object_position`  
        Type: geometry_msgs/msg/Point

### Run
In a terminal run the following commands:
`ros2 run cam2image_vm2ros cam2image --ros-args --params-file src/cam2image_vm2ros/config/cam2image.yaml`  
`ros2 run object_position object_position`

### Parameters
int `threshold` : Sets the brightness threshold.
bool `from_center` : Define whether the position coordinates should be relative to the center of the image.

### Core components 
* `topic_callback()`: Find the center of mass and publish to /output/object_position on each received image frame.