/*-----( Import needed libraries )-----*/
#include <Servo.h>

/*-----( Declare Constants and Pin Numbers )-----*/

// MOTORS
#define rMotorPin 10  // Can be changed 3,5,6,9,10,11 (NOW can be any pin including A0..A5)
#define lMotorPin 11

// PING SENSOR
const int rightPing = 6;
const int frontPing = 7;
const int leftPing = 8;

// IR SENSOR 
int sensorPin = A5; 
int sensorValue = 0;  // variable to store the value coming from the sensor
int initialValue = 0;
int maxChange = 20;

/*-----( Declare objects )-----*/
Servo lMotor;
Servo rMotor; 

/*-----( Declare Variables )-----*/
int forward = 1650;
int halt = 1500;
int backward = 1350;
int turnTime = 940;
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
  
  // NAVIGATE TO WALL
  int wallFind = selectWall();
  Serial.println(wallFind);
  steerToWall(wallFind);

  // INIT IR SENSOR
  pinMode(A5,INPUT)
  initialValue = analogRead(sensorPin);
  
}

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
//  forward4(); // goes forward 4 free //TODO recalibrate dist
//  lap4(); // goes in a counterclockwise square
//  hugWall(); // hugs wall
    fTraverse(); // forward, around obstacles
//    goAround();
}

//////////////////// LOOP OPTIONS ////////////////////

void fTraverse() {
  LRFevaluate();
  while (wallsFound == 0) {
    LRFevaluate();
    irScan();
    driveForward();
    if (Finches < 7) {wallsFound = 1;}
    if((sensorValue > initialValue + maxChange)) {wallsFound = 1; 
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

void hugWall() {
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
      wait(1150);
      doorFound = 1;
    }
    if(rightWall + 30 < Rinches && doorFound != 1){
      lMotor.writeMicroseconds(forward);
      rMotor.writeMicroseconds(backward);
      Serial.println("TURNING");
      wait(1150);
      doorFound = 1;
    }
    driveForward();
  }
  else{
    avoidObstacle();
  }
}

//////////////////// MANEUVERS ////////////////////

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
    while (Rinches < wallDist + 20) { // go until clear of obstacle
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
    while (Rinches < wallDist + 20) {
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
    while (Linches < wallDist + 20) { // go until clear of obstacle
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

void irScan() {
  sensorValue = analogRead(sensorPin);
}

void irGoAround() {
  // read the value from the sensor:
  int passed = 0; 
  irScan();
  stopBot();
  
  if (Linches > Rinches) { // go left
    left90(); // turn left
    driveForward();
    wait(2000);
    stopBot();
    wait(250);
    right90();
    driveBackward();
    wait(500);
  } 
  
  if (Linches <= Rinches) { // go right
    right90(); // turn left
    driveForward();
    wait(2000);
    stopBot();
    wait(250);
    left90();
    driveBackward();
    wait(500);
  }    
}
  
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
}

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

void driveBackward() {
  lMotor.writeMicroseconds(backward);
  rMotor.writeMicroseconds(backward);
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
    wait(1065);
    stopBot();
}

void right90() {
    if (relDir == 4) {relDir = 1;}
    else {relDir++;}
    lMotor.writeMicroseconds(forward);
    rMotor.writeMicroseconds(backward);
    wait(1065);
    stopBot();
}

void stopBot() {
    lMotor.writeMicroseconds(halt);
    rMotor.writeMicroseconds(halt);
}

