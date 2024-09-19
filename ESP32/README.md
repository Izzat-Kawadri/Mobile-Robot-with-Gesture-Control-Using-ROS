## Overview
This code allows an ESP32 to control a two-wheeled mobile robot by receiving velocity commands (`Twist` messages) via WiFi from 
a ROS (Robot Operating System) master node. 
The robot moves based on the commands it receives, such as moving forward, backward, or turning left and right, 
using the L298N motor driver to control two DC motors.

The ESP32 subscribes to the `/turtle1/cmd_vel` topic (which can be renamed depending on your ROS setup) and uses the received data
to control the robot's movements.

## Hardware Requirements
- **ESP32**: For wireless communication and robot control.
- **L298N Motor Driver**: To control the DC motors.
- **Two DC Motors**: For the robot's movement.
- **12V Lithium Battery**: To power the motors.
- **Wheels and Chassis**: The physical structure of the robot.
- **WiFi Network**: To connect the ESP32 with the ROS master.

### Pin Configuration
- **Motor Control Pins (L298N)**:
  - `ENA` (PWM): GPIO 12
  - `ENB` (PWM): GPIO 26
  - `IN1`: GPIO 32
  - `IN2`: GPIO 33
  - `IN3`: GPIO 25
  - `IN4`: GPIO 13

### WiFi Configuration
- **SSID**: `robot10`
- **Password**: `1234567890`

Ensure that your WiFi network matches this configuration, or update the values of `ssid` and `password` accordingly.

## ROS Setup
The ESP32 connects to a ROS master node over WiFi. Make sure the ROS master node is running and accessible on the specified IP address and port:
- **ROS Master IP Address**: `192.168.200.207`
- **ROS Master Port**: `11411`

Update the IP address and port if your ROS master is running on a different network or port.

### ROS Topic Subscription
The ESP32 subscribes to the `/turtle1/cmd_vel` topic for receiving velocity commands:
- **Linear Movement (X-axis)**: Controls forward and backward motion.
- **Angular Movement (Z-axis)**: Controls turning left and right.

The `Twist` messages are processed to move the robot based on the following logic:
- **Forward**: Positive linear velocity.
- **Backward**: Negative linear velocity.
- **Turn Left**: Positive angular velocity.
- **Turn Right**: Negative angular velocity.
- **Stop**: When both linear and angular velocities are zero.

## Code Breakdown

### 1. **WiFi Setup**
The ESP32 connects to the specified WiFi network in the `setup()` function:
```
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);

while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
}

```

Once connected, the ESP32 prints the IP address and proceeds to connect to the ROS master node.
### 2. **Motor Control Logic**

The robot's movement is controlled by the cmdVelCallback function, which processes the received Twist messages:

```

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_x = msg.linear.x;
  float angular_z = msg.angular.z;

  if (linear_x > 0) {
    // Move forward
  } else if (linear_x < 0) {
    // Move backward
  } else if (angular_z > 0) {
    // Turn left
  } else if (angular_z < 0) {
    // Turn right
  } else {
    // Stop motors
  }
}

```
- **Forward/Backward Movement**: Uses the IN1, IN2, IN3, and IN4 pins to control the motors.
- **Turning**: Adjusts the direction of the wheels to achieve the desired rotation.
- **Stopping**: Sets all motor control pins to LOW to stop the robot's movement.

### 3. **ROS Node Initialization**

The ESP32 is configured as a ROS node and subscribes to the /turtle1/cmd_vel topic:

```

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("turtle1/cmd_vel", &cmdVelCallback);

nh.initNode();
nh.subscribe(cmd_vel_sub);

```

The `nh.spinOnce()` call in the loop() ensures that the node continuously processes incoming messages.

### 4. **PWM Configuration**

PWM is used to control the motor speed:

```

ledcAttachPin(ENA, PWM1_Ch);
ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
ledcAttachPin(ENB, PWM2_Ch);
ledcSetup(PWM2_Ch, PWM2_Freq, PWM2_Res);

```

This setup ensures that both motors can be controlled independently through the L298N motor driver.

## How to Use
### 1. **Upload Code to ESP32**

- Connect your ESP32 to your computer.
- Upload the code using the Arduino IDE with the necessary ESP32 board support installed.
- Make sure your WiFi credentials are correct, and update the ROS master IP address if necessary.

### 2. **Run ROS Master**

On your ROS master machine, start the ROS core:

```

roscore
```

Ensure that the ESP32 can reach the ROS master by verifying network connectivity (both devices should be on the same network).

### 3. **Test the System**

Use a ROS publisher (such as a teleop node or custom script) to publish commands to the /turtle1/cmd_vel topic.
The ESP32 will control the robot based on the published Twist messages.

### 4. **Debugging**

Monitor the ESP32's serial output for logs to ensure it is receiving commands and moving as expected.

## Future Improvements

- **Obstacle Avoidance**: Integrate sensors like ultrasonic or infrared to stop the robot in case of obstacles.
- **Speed Control**: Implement dynamic speed control based on the magnitude of the linear and angular velocities.
- **Additional Features**: Add more advanced movement patterns or modes (e.g., line following, autonomous navigation).
- **Battery Monitoring**: Implement battery level monitoring to ensure safe operation.
