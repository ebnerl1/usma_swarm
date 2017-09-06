/*-----( Import needed libraries )-----*/
#include <Servo.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define rMotorPin  10  // Can be changed 3,5,6,9,10,11 (NOW can be any pin including A0..A5)
#define lMotorPin  11

/*-----( Declare objects )-----*/
Servo lMotor;
Servo rMotor; 

/*-----( Declare Variables )-----*/
int forward = 1650;
int halt = 1500;
int backward = 1350;
int turnTime = 940;
int forwardLimit = 0;
int turnLimit = 0;
int haltLimit = 0;

/* Declare constants */
long Linches, Rinches, Finches;
int wallsFound = 0;
int doorFound = 0;
int leftWall = 0;
int rightWall = 0;

//initializes variables for button pressing:
int leftButton;
int rightButton;
int lastLeftButton = 0;
int lastRightButton = 0;
long lastLeftDebounce = 0;
long lastRightDebounce = 0;
long debounceDelay = 50;
int wallFind = -1; //0 if find the left wall, 1 if find the right wall.


void setup()   /****** SETUP: RUNS ONCE ******/
{
  rMotor.attach(rMotorPin);
  lMotor.attach(lMotorPin);
  pinMode(2,INPUT); //turn right and find a wall
  pinMode(3,INPUT); //turn left and find a wall
  Serial.begin(9600);

  //finds which wall to navigate to:
  int wallFind = selectWall();
  Serial.println(wallFind);
  steerToWall(wallFind);
}

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  //forward4();
  //lap4();
  LRFevaluate();
  if(wallsFound == 0){
    wallsFound = 1;
    leftWall = Linches;
    rightWall = Rinches;
  }
  else if(Finches >= 20){
    Serial.print(leftWall);
    Serial.print(" ");
    Serial.print(rightWall);
    Serial.print("    ");
    Serial.print(Linches);
    Serial.print(" ");
    Serial.println(Rinches);
    if(leftWall + 30 < Linches && doorFound != 1){
      lMotor.writeMicroseconds(backward);
      rMotor.writeMicroseconds(forward);

      long time1 = millis();
      while(millis()-time1 < 1150){
      }
      
      doorFound = 1;
    }
    if(rightWall + 30 < Rinches && doorFound != 1){
      lMotor.writeMicroseconds(forward);
      rMotor.writeMicroseconds(backward);
      Serial.println("TURNING");

      long time1 = millis();
      while(millis()-time1 < 1150){
      }
      
      doorFound = 1;
    }
    driveForward();
  }
  else{
    avoidObstacle();
  }
}

void steerToWall(int wallFind){
  if(wallFind == 0){
    lMotor.writeMicroseconds(backward);
    rMotor.writeMicroseconds(forward);
    long time1 = millis();
    while(millis()-time1 < 1150){
      
    }
  }
  else{
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(backward);
    long time1 = millis();
    while(millis()-time1 < 1150){
      
    }
  }
  Finches = 40;
  while(Finches > 7){
    const int frontPing = 7;
    pinMode(frontPing, OUTPUT);
    digitalWrite(frontPing,LOW);
    delayMicroseconds(2);
    digitalWrite(frontPing,HIGH);
    delayMicroseconds(5);
    digitalWrite(frontPing,LOW);
    pinMode(frontPing,INPUT);
    Finches = microsecondsToInches(pulseIn(frontPing,HIGH));
    Serial.println(Finches);
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(forward);
  }
    
  lMotor.writeMicroseconds(halt);
  rMotor.writeMicroseconds(halt);
  if(wallFind == 1){
    lMotor.writeMicroseconds(backward);
    rMotor.writeMicroseconds(forward);
    long time1 = millis();
    while(millis()-time1 < 1150){
      
    }
  }
  else{
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(backward);
    long time1 = millis();
    while(millis()-time1 < 1150){
      
    }
  }
}

int selectWall(){
  while(wallFind == -1){
  int Lreading = digitalRead(2);
  int Rreading = digitalRead(3);
  if(Lreading != lastLeftButton){
    lastLeftDebounce = millis();
  }
  if(Rreading != lastRightButton){
    lastRightDebounce = millis();
  }
  if((millis()-lastLeftDebounce) > debounceDelay){
     if(Lreading != leftButton){
        leftButton = Lreading;
        if(leftButton = HIGH){
           wallFind = 1;
        }
     }
  }
  if((millis()-lastRightDebounce) > debounceDelay){
     if(Rreading != rightButton){
       rightButton = Rreading;
       if(rightButton = HIGH){
           wallFind = 0;
       }
     }
  }
  lastRightButton = Rreading;
  lastLeftButton = Lreading;
  }
  return wallFind;
}

void driveForward(){
  lMotor.writeMicroseconds(forward);
  rMotor.writeMicroseconds(forward);
}

void avoidObstacle(){
  lMotor.writeMicroseconds(halt);
  rMotor.writeMicroseconds(halt);
  
  lMotor.writeMicroseconds(forward);
  rMotor.writeMicroseconds(backward);

  long time1 = millis();
  while(millis()-time1 < 1150){
    
  }
}

long LRFevaluate(){
  const int rightPing = 6;
  const int frontPing = 7;
  const int leftPing = 8;
  
  pinMode(rightPing, OUTPUT);
  digitalWrite(rightPing,LOW);
  delayMicroseconds(2);
  digitalWrite(rightPing,HIGH);
  delayMicroseconds(5);
  digitalWrite(rightPing,LOW);
  pinMode(rightPing,INPUT);
  Rinches = microsecondsToInches(pulseIn(rightPing,HIGH));
  
  pinMode(leftPing, OUTPUT);
  digitalWrite(leftPing,LOW);
  delayMicroseconds(2);
  digitalWrite(leftPing,HIGH);
  delayMicroseconds(5);
  digitalWrite(leftPing,LOW);
  pinMode(leftPing,INPUT);
  Linches = microsecondsToInches(pulseIn(leftPing,HIGH));

  pinMode(frontPing, OUTPUT);
  digitalWrite(frontPing,LOW);
  delayMicroseconds(2);
  digitalWrite(frontPing,HIGH);
  delayMicroseconds(5);
  digitalWrite(frontPing,LOW);
  pinMode(frontPing,INPUT);
  Finches = microsecondsToInches(pulseIn(frontPing,HIGH));
}

long microsecondsToInches(long microseconds){
  return microseconds / 74 / 2;
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

