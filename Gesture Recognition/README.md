## Overview
This Python script enables gesture-based control of a robot using hand gestures captured from a live video feed.
The system uses **MediaPipe** for hand gesture recognition and **ROS** to send commands to the robot,
specifically publishing velocity commands to the `cmd_vel` topic. 
This project used to wirelessly control a robot, based on hand movements captured via a laptop or webcam.

The recognized gestures are translated into commands for the robot to move forward, backward, turn left, turn right, or stop.

## Requirements
### 1. ROS Noetic
Ensure you have ROS Noetic installed and properly set up on your system. You can follow the official 
[ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation) to install it on your machine.

### 2. Python Libraries
The script requires the following Python libraries:
- **OpenCV**: For capturing video from the webcam.
- **MediaPipe**: For detecting and tracking hand landmarks.
- **rospy**: For interfacing with ROS and publishing commands.
- **geometry_msgs**: To publish velocity commands (`Twist` messages).

Install the necessary Python libraries using the following commands:
```
pip install opencv-python mediapipe
sudo apt-get install ros-noetic-geometry-msgs
```

### ROS Setup

Before running the script, ensure that the ROS master node is running,
and your robot or simulation is configured to receive commands on the `/cmd_vel` topic.

If you are using Turtlesim for debugging, you can start it by running:

```

rosrun turtlesim turtlesim_node
```

## How It Works
### 1. Hand Gesture Detection:
The script uses MediaPipe to detect hand landmarks from the webcam feed.
Based on the relative position of the hand's width and height in the video frame, specific gestures are recognized,
such as moving the hand to the left, right, up, or down.

### 2. Gesture Mapping:

The script defines five basic robot actions:

   - **Move Forward** : When the hand moves up in the frame.
   - **Move Backward**: When the hand moves down.
   - **Turn Left**: When the hand moves to the left side.
   - **Turn Right**: When the hand moves to the right side.
   - **Stop**: When no gestures are detected or when the hand is in a neutral position.

### 3. Publishing to ROS:

The recognized gesture is mapped to a Twist message, which is published to the `/turtle1/cmd_vel` (or `/cmd_vel` for robot) topic.
The robot will then move according to the received gesture-based command.

---

## Code Explanation
### ROS Node Initialization

```

rospy.init_node('gesture_control')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

```

This section initializes the ROS node and creates a publisher to the `/cmd_vel` topic, 
where the robot subscribes to receive velocity commands.

### Gesture Recognition

The script uses MediaPipe to process video frames from the webcam:

```
with mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        success, image = cap.read()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

```

It captures the video feed and converts the frame to RGB format.
The processed hand landmarks are used to calculate hand width and height, which are the basis for gesture detection.

### Gesture Logic

Based on the position of the hand in the frame, the script determines the movement gesture:

```

if hand_width > hand_height:
    if hand_width > GESTURE_THRESHOLD:
        if min_x < GESTURE_THRESHOLD:
            current_gesture = gestures['TURN_LEFT']
        elif max_x > 1 - GESTURE_THRESHOLD:
            current_gesture = gestures['TURN_RIGHT']
        else:
            current_gesture = gestures['STOP']
else:
    if hand_height > GESTURE_THRESHOLD:
        if min_y < GESTURE_THRESHOLD:
            current_gesture = gestures['MOVE_FORWARD']
        elif max_y > 1 - GESTURE_THRESHOLD:
            current_gesture = gestures['MOVE_BACKWARD']
        else:
            current_gesture = gestures['STOP']

```

If the hand moves left or right, the robot will turn accordingly.
If the hand moves up or down, the robot will move forward or backward.
The robot will stop if no significant movement is detected.

### Command Publishing

Once a gesture is recognized, a corresponding Twist message is published to the /cmd_vel topic:

```

if current_gesture:
    pub.publish(current_gesture)

```

This allows the robot to react in real-time to hand gestures.

### Visualization

The script also visualizes the detected hand landmarks in the video feed for better debugging:

```

mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
cv2.imshow('Gesture Control', image)

```

It uses OpenCV to display the video with hand landmarks drawn on the frame.

### Exiting the Program

The script listens for the 'q' key to safely close the webcam and ROS nodes:

```

if cv2.waitKey(1) & 0xFF == ord('q'):
    break

```

### Running the Script

To run the script, ensure your ROS master is running and execute the following command:

```

python3 gesture_control.py

```

This will start capturing gestures from your webcam and controlling the robot via ROS.

### Optional: Using Turtlesim for Testing

You can test the script with Turtlesim before deploying it on an actual robot. Ensure the turtle is listening on the /turtle1/cmd_vel topic and start Turtlesim by running:

```

rosrun turtlesim turtlesim_node

```

## Future Improvements

   - **Additional Gestures**: More complex gestures could be added for finer control (e.g., hand rotations, more movement options).
   - **Gesture Accuracy**: Fine-tuning the gesture recognition thresholds or using machine learning models for higher accuracy.
   - **Obstacle Avoidance**: Integrate sensors with the robot for automatic stopping or avoidance when obstacles are detected.

This script combines MediaPipe and ROS to create a gesture-based control system for robots, demonstrating real-time interaction between a camera, hand gestures, and a robotic platform.
