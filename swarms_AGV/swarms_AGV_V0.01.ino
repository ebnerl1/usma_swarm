/*-----( Import needed libraries )-----*/
#include <Servo.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define rMotorPin  10  // Can be changed 3,5,6,9,10,11 (NOW can be any pin including A0..A5)
#define lMotorPin  11

/*-----( Declare objects )-----*/
Servo lMotor;
Servo rMotor; 

/*-----( Declare Variables )-----*/
int forward = 2000;
int halt = 1500;
int backward = 1000;
int forwardLimit = 0;
int turnLimit = 0;
int haltLimit = 0;



void setup()   /****** SETUP: RUNS ONCE ******/
{
  rMotor.attach(rMotorPin);
  lMotor.attach(lMotorPin);
}

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  //forward4();
  lap4();
}

void lap4() {
  if (forwardLimit < 110) {
  forward4();
  }
  if (forwardLimit == 110) {
    halt1();
    forwardLimit++;
  }
  if (forwardLimit > 110) {
    right90();
  }
  if (turnLimit >= 35) {
    halt1();
    turnLimit = 0;
    haltLimit = 0;
    forwardLimit = 0;
  }
}

void forward4() {
  if (forwardLimit < 110) {
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(forward);
    forwardLimit++;
    delay(10);
  }
  else {
    lMotor.writeMicroseconds(halt);
    rMotor.writeMicroseconds(halt);
  }
}

void left90() {
  if (turnLimit < 35) {
    backward = 1000;
    forward = 2000;
    lMotor.writeMicroseconds(backward);
    rMotor.writeMicroseconds(forward);
    turnLimit++;
    delay(10);
  }
  else {
    lMotor.writeMicroseconds(halt);
    rMotor.writeMicroseconds(halt);
  }
}

void right90() {
  if (turnLimit < 35) {
    backward = 1000;
    forward = 2000;
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(backward);
    turnLimit++;
    delay(10);
  }
  else {
    lMotor.writeMicroseconds(halt);
    rMotor.writeMicroseconds(halt);
  }
}

void halt1() {
  if (haltLimit < 15) {
    lMotor.writeMicroseconds(halt);
    rMotor.writeMicroseconds(halt);
    haltLimit++;
    delay(10);
  }
}

