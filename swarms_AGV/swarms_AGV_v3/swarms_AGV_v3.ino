/*-----( Import needed libraries )-----*/
#include <Servo.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define rMotorPin 10  // Can be changed 3,5,6,9,10,11 (NOW can be any pin including A0..A5)
#define lMotorPin 11

const int rightPing = 6;
const int frontPing = 7;
const int leftPing = 8;

/*-----( Declare objects )-----*/
Servo lMotor;
Servo rMotor; 

/*Declare Pins*/
int sensorPin0 = A0;//left    select the input pin for the potentiometer 
int sensorPin1 = A1;//front
int sensorPin2 = A2;
double sensorValue0 = 0;  // variable to store the value coming from the sensor
double sensorValue1 = 0;
double sensorValue2 = 0;
int orientation = 0;

/*-----( Declare Variables )-----*/
int forward = 1650;
int halt = 1500;
int backward = 1350;
int leftTurnTime = 1050;
int rightTurnTime = 990;
int fourFeet = 110;
int wallDist;
int relDir = 1;

/* Declare constants */
long Linches, Rinches, Finches;
int wallsFound = 0; // true false statements
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
  pinMode(sensorPin0, INPUT);
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  
//  finds which wall to navigate to:
  int wallFind = selectWall();
  Serial.println(wallFind);
  steerToWall(wallFind);

  doorEvaluate(wallFind);
  if(wallFind == 0){
    leftWall = Linches;
  }
  else{
    rightWall = Rinches;
  }
}

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
//  forward4(); // goes forward 4 free //TODO recalibrate dist
//  lap4(); // goes in a counterclockwise square
    if (doorFound == 0) {hugWall();} // hugs wall
    else if (doorFound == 1) {
      findLight();
      //fTraverse(); //forward around obstacles
    } 
}

//////////////////// LOOP OPTIONS ////////////////////

void fTraverse() {
  lightScan();
  LRFevaluate();
  
  while (wallsFound == 0) {
    LRFevaluate();
    lightScan();
    
    if (doorFound == 0) {hugWall();}
    if (Finches < 7) {wallsFound = 1;}
    else if (doorFound == 1) {
      driveForward();
      findLight();
    }
  }
  
  if (wallsFound == 1) {
    // turn left or right based on obstacle, drive until past obstacle
    goAround();
    wallsFound = 0;
  }
}

void lap4() {
  forward4();
  wait(500);
  left90();
  wait(500);
}

/*evaluates the ping sensors to determine if there is an obstacle in the way 
 * or if the door has been found. Only pings left or right depending on which
 * wall it is hugging. wallFind = 0: left wall. wallFind = 1: right wall.
 */
long doorEvaluate(int wallFind){

  if(wallFind == 1){
    pinMode(rightPing, OUTPUT);
    digitalWrite(rightPing,LOW);
    delayMicroseconds(2);
    digitalWrite(rightPing,HIGH);
    delayMicroseconds(5);
    digitalWrite(rightPing,LOW);
    pinMode(rightPing,INPUT);
    Rinches = microsecondsToInches(pulseIn(rightPing,HIGH));
  }

  if(wallFind == 0){
    pinMode(leftPing, OUTPUT);
    digitalWrite(leftPing,LOW);
    delayMicroseconds(2);
    digitalWrite(leftPing,HIGH);
    delayMicroseconds(5);
    digitalWrite(leftPing,LOW);
    pinMode(leftPing,INPUT);
    Linches = microsecondsToInches(pulseIn(leftPing,HIGH));
  }
  
  pinMode(frontPing, OUTPUT);
  digitalWrite(frontPing,LOW);
  delayMicroseconds(2);
  digitalWrite(frontPing,HIGH);
  delayMicroseconds(5);
  digitalWrite(frontPing,LOW);
  pinMode(frontPing,INPUT);
  Finches = microsecondsToInches(pulseIn(frontPing,HIGH));
}


void hugWall() {
  doorEvaluate(wallFind);

  /* if no obstacles are in the car's path*/
  if(Finches >= 20){
    Serial.print(leftWall);
    Serial.print(' ');
    Serial.println(Linches);
    if(leftWall + 30 < Linches && doorFound == 0 && wallFind == 0){
      left90();
      doorFound = 1;
      driveForward2(4000);
    }
    if(rightWall + 30 < Rinches && doorFound == 0 && wallFind == 1){
      right90();
      doorFound = 1;
      driveForward2(4000);
    }
    
    driveForward();
  }
  else{
    avoidObstacle();
  }
}

//////////////////// MANEUVERS ////////////////////

void lightScan() {
  sensorValue0 = analogRead(sensorPin0);
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
}
  

void findLight(){
  lightScan();
  driveForward();
  Serial.println(sensorValue1);
  if (sensorValue0 > sensorValue1 && sensorValue0 > sensorValue2) { // if more light left
    Serial.println("Turn Left");
    left45();
    orientation = orientation - 45;
  }
  else if (sensorValue2 > sensorValue0 && sensorValue2 > sensorValue1) { // if more light right
    Serial.println("Go Right");
    right45();
    orientation = orientation + 45;
  }
  else {
    Serial.println("Go Straight");
    driveForward();
  }
}

void steerToWall(int wallFind){
  if(wallFind == 0){ // if steer to left wall
  left90();
  }
  else{
    right90();
  }
  Finches = 40;
  while(Finches > 5){ // Ping wall until closer than 7
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
    driveForward();
  }
    
  stopBot(); // stop once wall found
  if(wallFind == 0){
    right90();
  }
  else{
    left90();
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

void goAround() {
  stopBot();
  wallDist = Finches;
  if (Linches > Rinches) { // go left
    left90(); // turn left
    int sideStart = millis(); // begin timing width of obstacle
    while (Rinches < wallDist + 30) { // go until clear of obstacle
      driveForward();
      LRFevaluate();
    }
    wait(250); // go a little bit extra to clear the object
    int sideEnd = millis() - sideStart; // necessary traverse
    
    right90(); // forward traverse obstacle
    wait(200);
    driveForward();
    wait(500);
    LRFevaluate();
    while (Rinches < wallDist + 30) {
      driveForward();
      LRFevaluate();
    }
    driveForward(); // go a little bit extra to clear the object
    wait(250);
    
    right90();
    driveForward();
    wait(sideEnd);
    left90();
  }

  if (Linches <= Rinches) { // go right
    right90(); // turn right
    int sideStart = millis(); // begin timing width of obstacle
    while (Linches < wallDist + 30) { // go until clear of obstacle
      driveForward();
      LRFevaluate();
    }
    wait(250); // go a little bit extra to clear the object
    int sideEnd = millis() - sideStart; // necessary traverse
    
    left90(); // forward traverse obstacle
    wait(200);
    driveForward();
    wait(500);
    LRFevaluate();
    while (Linches < wallDist + 20) {
      driveForward();
      LRFevaluate();
    }
    driveForward(); // go a little bit extra to clear the object
    wait(250);
    
    left90();
    driveForward();
    wait(sideEnd);
    right90();
  }
}

void avoidObstacle(){
  lMotor.writeMicroseconds(halt);
  rMotor.writeMicroseconds(halt);

  
  lMotor.writeMicroseconds(forward);
  rMotor.writeMicroseconds(backward);

  wait(1150);
}

//////////////////// SENSORS ////////////////////

long LRFevaluate(){ // pulses LRF in order to read inches
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

//////////////////// Basic Movement Functions ////////////////////

void wait(int t) {
  long t2 = millis();
  while(millis() - t2 < t){
    // do nothing    
  }
}

void driveForward(){
  lMotor.writeMicroseconds(forward);
  rMotor.writeMicroseconds(forward);
}

void driveForward2(int t){
  lMotor.writeMicroseconds(forward);
  rMotor.writeMicroseconds(forward);
  long t1 = millis();
  while(millis()-t1 < t){
    
  }
}

void forward4() {
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(forward);
    wait(2000);
    stopBot();
}

void left90() {
    if (relDir == 1) {relDir = 4;}
    else {relDir--;}
    lMotor.writeMicroseconds(backward);
    rMotor.writeMicroseconds(forward);
    wait(leftTurnTime);
    stopBot();
}

void right90() {
    if (relDir == 4) {relDir = 1;}
    else {relDir++;}
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(backward);
    wait(rightTurnTime);
    stopBot();
}

void left45() {
    if (relDir == 1) {relDir = 4;}
    else {relDir--;}
    lMotor.writeMicroseconds(backward);
    rMotor.writeMicroseconds(forward);
    wait(leftTurnTime/2);
    stopBot();
}

void right45() {
    if (relDir == 4) {relDir = 1;}
    else {relDir++;}
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(backward);
    wait(rightTurnTime/2);
    stopBot();
}

void stopBot() {
    lMotor.writeMicroseconds(halt);
    rMotor.writeMicroseconds(halt);
}

