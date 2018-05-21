#include <Servo.h>
#define rMotorPin 10
#define lMotorPin 11

Servo lMotor;
Servo rMotor;

int sensorPin = A5;    // select the input pin for the potentiometer
     // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
int initialValue = 0;
int maxChange = 20;

int forward = 1600;
int halt = 1500;
int backward = 1350;

void setup() {
  // put your setup code here, to run once:
  rMotor.attach(rMotorPin);
  lMotor.attach(lMotorPin);
  pinMode(A5,INPUT);
  Serial.begin(9600);
  initialValue = analogRead(sensorPin);
}

void loop() {
   // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  int passed = 0;
  // turn the ledPin o
  // stop the program for <sensorValue> milliseconds
  // turn the ledPin off:
  Serial.println(sensorValue);
  if((sensorValue < initialValue + maxChange)){
      Serial.println(sensorValue);
      lMotor.writeMicroseconds(forward);
      rMotor.writeMicroseconds(forward);
      Serial.println("Driving Forward");
  }
  else{
    lMotor.writeMicroseconds(halt);
    rMotor.writeMicroseconds(halt);
    long time1 = millis();
    while(millis()-time1 < 2000){
      
    }
    passed = 1;
    lMotor.writeMicroseconds(backward);
    rMotor.writeMicroseconds(backward);
    time1 = millis();
    while(millis()-time1 < 1000){
      
    }
  }
  // put your main code here, to run repeatedly:
}
