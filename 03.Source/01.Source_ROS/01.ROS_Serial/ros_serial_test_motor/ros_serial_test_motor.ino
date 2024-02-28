#include <ros.h>
#include <std_msgs/Int32.h>

#define FULSE_TARGET 80000
#define TIME_FULSE 100

// Define pin numbers
const int pulsePin = 8;
const int dirPin = 9;
int speed_motor = 20;

ros::NodeHandle nh;

void motorCallback(const std_msgs::Int32& msg) {
  int speed = msg.data;
  speed_motor = speed;
}

ros::Subscriber<std_msgs::Int32> sub("motor_speed", &motorCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  pinMode(pulsePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);
}

void loop() {
  nh.spinOnce();
  // delay(10);
  digitalWrite(pulsePin, HIGH);
  delayMicroseconds(speed_motor);
  digitalWrite(pulsePin, LOW);
  delayMicroseconds(speed_motor);
}
