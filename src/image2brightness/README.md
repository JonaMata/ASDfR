Package image2brightness
-----------------------------------------------
### Description 
This package implements a node that checks the average brightness of an image against a threshold to check if the lights are on or off.

### Inputs
`/image`  
        Type: sensor_msgs/msg/Image  

### Outputs
`/light_on`  
        Type: std_msgs/msg/Bool

### Run
In a terminal run the following commands:
`ros2 run cam2image_vm2ros cam2image --ros-args --params-file src/cam2image_vm2ros/config/cam2image.yaml`  
`ros2 run image2brightness image2brightness`

### Parameters
int `threshold` : Sets the brightness threshold.

### Core components 
* `topic_callback()`: Calculate and check the average brightness of an image and publish to /light_on on every received image frame.