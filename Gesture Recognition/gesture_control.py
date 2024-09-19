#!/usr/bin/env python3

import cv2
import mediapipe as mp
import rospy
from geometry_msgs.msg import Twist, Vector3

# ROS publishers for robot control
rospy.init_node('gesture_control')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

# Initialize MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# Define gesture thresholds
GESTURE_THRESHOLD = 0.5

# Define gesture actions
gestures = {
    'MOVE_FORWARD': Twist(linear=Vector3(x=0.2)),
    'MOVE_BACKWARD': Twist(linear=Vector3(x=-0.2)),
    'TURN_RIGHT': Twist(angular=Vector3(z=-0.2)),
    'TURN_LEFT': Twist(angular=Vector3(z=0.2)),
    'STOP': Twist()
}

# Initialize gesture state
current_gesture = None

# OpenCV video capture
cap = cv2.VideoCapture(0)

with mp_hands.Hands(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Convert the image to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Process the image with MediaPipe
        results = hands.process(image)


        # Check if hands are detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Get landmarks for each hand
                x_values = [landmark.x for landmark in hand_landmarks.landmark]
                y_values = [landmark.y for landmark in hand_landmarks.landmark]

                # Calculate gesture features
                max_x = max(x_values)
                min_x = min(x_values)
                max_y = max(y_values)
                min_y = min(y_values)

                # Calculate hand width and height
                hand_width = max_x - min_x

                hand_height = max_y - min_y


                # Perform gesture recognition
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
        else:
            current_gesture = None

        # Publish the current gesture
        if current_gesture:
            print(current_gesture)
            pub.publish(current_gesture)

        # Draw hand landmarks on the image
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # Display the image with hand landmarks
        cv2.imshow('Gesture Control', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
