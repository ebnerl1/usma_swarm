#include <ros.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);
ros::NodeHandle nh;

int val;
int tempPin = 1;

void setup() {
  Serial.begin(9600);
}

void loop() {
  val = analogRead(tempPin);
  float mv = (val/1024.0)*5000;
  float cel = mv/10;
  Serial.print("The temperature is: "); Serial.print(cel); Serial.print(" degrees Celsius");
  Serial.println(" ");
  delay(1000);
}
