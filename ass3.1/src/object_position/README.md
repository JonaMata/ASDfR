Package object_position
-----------------------------------------------
### Description 
This package implements a node that finds the position of the largest green blob-shaped object in the image. It uses OpenCV to create and clean a color green mask and then runs a SimpleBlobDetector. The z coordinate of the output's Point is the size of the blob.

### Inputs
`/image`  
        Type: sensor_msgs/msg/Image  

### Outputs
`/output/object_position`  
        Type: geometry_msgs/msg/Point

### Run
In a terminal run the following commands:
`ros2 run cam2image_vm2ros cam2image --ros-args --params-file src/sequence_controller/config/cam2image_relbot.yaml`  
`ros2 run object_position object_position`

### Parameters
bool `from_center` : Define whether the position coordinates should be relative to the center of the image.
bool `show_mask` : Define whether the masked frame with a circle around the detected blob should be shown with `imshow`. This significantly slows processing when used over X forwarding.

### Core components 
* `topic_callback()`: Find the green blob's position and publish to /output/object_position on each received image frame.