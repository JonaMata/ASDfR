# Assignment 1

Most assigments need a webcam feed on the /image topic.
Make sure to keep `videoserver.py` running in a terminal on the webcam host and the following in a terminal on the ros2 host: `ros2 run cam2image_vm2ros cam2image --ros-args --params-file src/cam2image_vm2ros/config/cam2image.yaml`

## Assignment 1.1.2

Run:

`ros2 run image2brightness image2brightness`

Location in code: Image2Brightness class in image2brightness.cpp

## Assignment 1.1.3

Run:

`ros2 run image2brightness image2brightness --ros-args -p threshold:=70`

NOTE: The threshold can be changed depending on the ambient light situation.

And to change the threshold during runtime run the following in another terminal:

`ros2 param set /image2brightness threshold 80`


Location in code: Image2Brightness class in image2brightness.cpp

## Assignment 1.1.4

Run

`ros2 run object_position object_position`

Location in code: ObjectPosition class in object_position.cpp

## Assignment 1.2.1

Run

`ros2 launch sequence_controller sequence_controller_launch.py`

Location in code: SequenceController class in sequence_controller.cpp

## Assignment 1.2.2

Run

`ros2 launch sequence_controller sequence_controller_object_launch.py`

Location in code: SequenceController class in sequence_controller.cpp

## Assignment 1.2.3

Run

`ros2 launch object_tracker object_tracker_launch.py`

Location in code: ObjectTracker class in object_tracker.cpp