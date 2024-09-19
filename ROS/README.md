# ROS Workspace for Mobile Robot with Gesture Control

## Overview
This ROS workspace contains the necessary ROS packages and nodes for controlling a mobile robot using hand gestures or keyboard inputs. 
The workspace integrates with the ESP32 controller and the gesture recognition script to enable wireless control of the robot.
The system uses ROS Noetic to facilitate communication between different components,
including motor control and hand gesture recognition.


## Getting Started

### 1. Build the ROS Workspace
First, you need to build the ROS workspace using `catkin_make`:

```
cd catkin_ws
catkin_make
```

### 2. Source the Workspace

Once the workspace is built, source it to ensure that all ROS packages are available for use:

```
source devel/setup.bash
```
### 3. Start the Necessary Nodes

```
rosrun rosserial_python serial_node.py tcp

rosrun gesture_robot test_publisher.py 

rosrun turtlesim turtlesim_node 

rosrun turtlesim turtle_teleop_key 

```

This will start the necessary nodes, including the one that subscribes to the `cmd_vel` topic and controls the robot's movement.

## Nodes Overview
### 1. `cmd_vel` Topic Subscriber

The robot is controlled by velocity commands published to the `cmd_vel` topic. 
The ESP32 subscribes to this topic to receive commands and control the motors.

### 2. Teleop Keyboard Control (Optional)

You can manually control the robot using keyboard inputs through the Teleop node. To run it, use:

```

rosrun  teleop_keyboard.py

```


This will allow you to move the robot forward, backward, left, or right using keyboard keys.

## Dependencies

Ensure that you have the following dependencies installed for ROS Noetic:
-    ros-noetic-roscpp
-   ros-noetic-geometry-msgs
-    ros-noetic-std-msgs
-   ros-noetic-turtlesim (for simulation and debugging)

You can install them using:
```

sudo apt-get install ros-noetic-roscpp ros-noetic-geometry-msgs ros-noetic-std-msgs ros-noetic-turtlesim

```

## Troubleshooting

ROS Environment Not Set: If the ROS environment variables are not set, try sourcing the workspace again:

 ```  
source devel/setup.bash
```

- **No Movement**: Ensure that the ESP32 is properly connected to ROS and that it is receiving commands from the `cmd_vel` topic.

- **Turtlesim Debugging**: Use the turtlesim package to simulate and debug the robot's movement. Launch it by running:

```
    rosrun turtlesim turtlesim_node
```

## Future Improvements

- **More Nodes**: Adding more complex functionality like obstacle detection, SLAM, or more control methods.
-   **Improved Launch Files**: Refining the launch process to include more features and options.
-    **Hardware Integration**: Expanding the ROS packages to support additional sensors or hardware.

This ROS workspace is part of a larger project involving a mobile robot controlled by hand gestures and wireless communication. The workspace can be expanded and customized for more advanced features.
