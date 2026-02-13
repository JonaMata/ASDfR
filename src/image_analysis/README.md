Package image_analysis
-----------------------------------------------
### Description 
This package implements image anlysis functions.

### Inputs
`/image`  
        Type: sensor_msgs/msg/Image  

### Outputs
`/output/camera_position`  
        Type: geometry_msgs/msg/PointStamped
        X,Y position of camera centre on input image

### Run
In a terminal run either of the following commands:
`ros2 run relbot_simulator relbot_simulator`  
`ros2 relbot_launch relbot_system.launch.py`

### Parameters
int `threshold` : Sets the brightness threshold.

### Core components 
TODO
* `dynamics_timer_callback()`: Calls and runs the model every simulation timestep
* `webcam_topic_callback()`: Ingests and passes on webcam image stream
* `CreateCVSubimage()`: Creates sub-image which is then published on /output/moving_camera