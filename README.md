# my_beginner_tutorials

## Overview
This pacakage contains the beginner tutorials for ROS 2 Humble assigned in the ROS 2 Programming Assignment 1 - Publisher/Subscriber assignment in ENPM 700 RO01 Fall 2025. The talker node publishes "Marcus' custom string message n" where n is a counter of the number of messages sent. The listener subscribes to topic and logs the messages it recieves.

## Contributors
Marcus Hurt

## Dependencies
- ROS2 Humble

## Build/Run Steps
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

# Run the listener
ros2 run beginner_tutorials listener

# In a new terminal from the workspace root, source and run the talker
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run beginner_tutorials talker
```