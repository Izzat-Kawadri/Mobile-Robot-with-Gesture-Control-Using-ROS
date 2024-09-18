#include <WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#define PWM1_Ch   2 
#define PWM1_Res   8
#define ENA 12
#define ENB  26
#define PWM1_Freq  1000
#define PWM2_Ch    1
#define PWM2_Res   8
#define PWM2_Freq  1000
const char* ssid     = "robot10"; // Wifi name
const char* password = "1234567890"; // Wifi password

IPAddress server(192, 168, 200, 207); // ROS Master IP address
const uint16_t serverPort = 11411; // ROS Master port

// Motor driver pin configuration
const int input1 = 32;
const int input2 = 33;
const int input3 = 25;
const int input4 = 13;


void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // Implement your robot control logic based on Twist messages here
  float linear_x = msg.linear.x;
  float angular_z = msg.angular.z;

  // Example: Print received Twist values
  Serial.print("Linear X: ");
  Serial.println(linear_x);
  Serial.print("Angular Z: ");
  Serial.println(angular_z);

  // Motor control logic using L298N motor driver
  if (linear_x > 0) {
    // Move forward
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
    digitalWrite(input3, LOW);
    digitalWrite(input4, HIGH);
  
  } else if (linear_x < 0) {
    // Move backward
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
    digitalWrite(input3, HIGH);
    digitalWrite(input4, LOW);
   
  }

  else if (angular_z < 0) {
    // Turn right
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
    digitalWrite(input3, LOW);
    digitalWrite(input4, LOW);
   
  } else if (angular_z > 0) {
    // Turn left
    digitalWrite(input1, LOW);
    digitalWrite(input2, LOW);
    digitalWrite(input3, LOW);
    digitalWrite(input4, HIGH);
 
  } else{
    // Stop motors for angular movement
     Serial.println("Stopping motors");
    digitalWrite(input1, LOW);
    digitalWrite(input2, LOW);
    digitalWrite(input3, LOW);
    digitalWrite(input4, LOW);
  }
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("turtle1/cmd_vel", &cmdVelCallback);

void setup() {
  ledcAttachPin(ENA, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  ledcAttachPin(ENB ,PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM2_Freq, PWM2_Res);

  ledcWrite(PWM1_Ch, 100);
  ledcWrite(PWM2_Ch, 100); 

  Serial.begin(115200);

  // Configure motor driver pins
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Connecting to ROS...");
  nh.getHardware()->setConnection(server, serverPort);
  
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
