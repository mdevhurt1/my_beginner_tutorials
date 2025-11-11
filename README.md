# my_beginner_tutorials

## Overview
This pacakage contains the beginner tutorials for ROS 2 Humble assigned in the ROS 2 Programming Assignment 2 - Services, Logging, and Launch files assignment in ENPM 700 RO01 Fall 2025. The talker node publishes a custom message based on the service_flag_ and can be started/stopped via the publishing_flag parameter at launch. The listener subscribes to topic and logs the messages it recieves.

## Contributors
Marcus Hurt

## Dependencies
- ROS2 Humble

## Build/Run Steps
Remember to source ROS and the overlay in any new terminals.

```bash
# Source the ROS 2 underlay
source /opt/ros/humble/setup.bash

# Make a colcon workspace for ROS 2 and enter the src/ directory
mkdir -p ros_ws/src
cd ros_ws/src

# Clone the package into the src/ directory
git clone https://github.com/mdevhurt1/my_beginner_tutorials.git

# Navigate to the workspace root and build with colcon
cd ..
colcon build

# Source the overlay
source install/local_setup.bash

# Run the talker node
ros2 run beginner_tutorials talker_node

# Run the listener node
ros2 run beginner_tutorials listener_node

# Run the talker and listener from the launch file
ros2 launch beginner_tutorials tutorial.launch.py

# Run the talker and listener with cmd line modifications
ros2 launch beginner_tutorials tutorial.launch.py publishing_flag:=false

# Call the talker set_flag service
ros2 service call /set_flag std_srvs/srv/SetBool {'data: true'}
```

## Deliverables
### rqt_console screenshot
![rqt_console](screenshots/rqt_console.png)