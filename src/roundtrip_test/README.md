Package roundtrip_test
-----------------------------------------------
### Description 
This package implements two nodes to test the timing of roundtrip messaging between two ros2 nodes.

### Seq13 Node

#### Inputs
`/roundtrip/end`  
        Type: std_msgs/msg/Int64

#### Outputs
`/roundtrip/start`  
        Type: std_msgs/msg/Int64

### Loop13 Node

#### Inputs
`/roundtrip/start`  
        Type: std_msgs/msg/Int64

#### Outputs
`/roundtrip/end`  
        Type: std_msgs/msg/Int64



### Run
In a terminal the following sets of command:

`ros2 launch roundtrip_test roundtrip_test_launch.py`

This should result in a file called `seq13.csv` with the start and end times of each roundtrip message.


### Core components 
* `Seq13()`: Publishes the current timestamp to `/roundtrip/start` on a timer interval. Receives on `/roundtrip/end` and stores the message and time of receiving.
* `Loop13()`: Forwards messages received on `/roundtrip/start` to `/roundtrip/end`.