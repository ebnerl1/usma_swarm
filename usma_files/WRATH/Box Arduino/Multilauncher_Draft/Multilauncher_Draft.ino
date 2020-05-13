//READ ME
//This is a template of an expanded version of Rapid Quick Launcher Code. This is setup
//to work for up to 4 launchers. this is a draft, needs edits to be useable. includes
//lcd and launcher manipulation



#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
int sensorPin = A8;
float batteryThreshold = 11.2; //Dependent on Battery Type, this is 4xAA
float batteryMaximum = 13.2; //Dependent on Battery Type

#include <Servo.h>

//Servos for the first launcher
Servo Box1Lock1;
Servo Box1Lock2;

//Servos for the second launcher
Servo Box2Lock1;
Servo Box2Lock2;

//Servos for the third launcher
Servo Box3Lock1;
Servo Box3Lock2;

//Servos for the fourth launcher
Servo Box4Lock1;
Servo Box4Lock2;

//LEDs for the first launcher
int Box1Unlocked = 30;
int Box1UnlockedRead = 31;
int Box1Locked = 32;
int Box1LockedRead = 33;

//LEDs for the second launcher
int Box2Unlocked = 34;
int Box2UnlockedRead = 35;
int Box2Locked = 36;
int Box2LockedRead = 37;

//LEDs for the third launcher
int Box3Unlocked = 38;
int Box3UnlockedRead = 39;
int Box3Locked = 40;
int Box3LockedRead = 41;

//LEDs for the fourth launcher
int Box4Unlocked = 42;
int Box4UnlockedRead = 43;
int Box4Locked = 44;
int Box4LockedRead = 45;

//Switch for the first launcher
int Switch1 = 46;
int Switch1Read = 47;

//Switch for the second launcher
int Switch2 = 48;
int Switch2Read = 49;

//Switch for the third launcher
int Switch3 = 50;
int Switch3Read = 51;

//Switch for the fourth launcher
int Switch4 = 52;
int Switch4Read = 53;

//Initial and Final Position for ALL Servos
int ServoStart = 20;
int ServoEnd = 0;

//Pin to open launcher 1
int Latch1 = 22;

//Pin to open launcher 2
int Latch2 = 23;

//Pin to open launcher 3
int Latch3 = 24;

//Pin to open launcher 4
int Latch4 = 25;


void setup() {
  lcd.begin();
  lcd.clear();
  lcd.backlight();
  pinMode(sensorPin, INPUT);

  // put your setup code here, to run once:
  //Assign Pins for launcher 1 Servos
  Box1Lock1.attach(13);
  Box1Lock2.attach(12);

  //Assign Pins for launcher 2 Servos
  Box2Lock1.attach(11);
  Box2Lock2.attach(10);

  //Assign Pins for launcher 3 Servos
  Box3Lock1.attach(9);
  Box3Lock2.attach(8);

  //Assign Pins for launcher 4 Servos
  Box4Lock1.attach(7);
  Box4Lock2.attach(6);

  //Assign Launcher 1 Pinmodes
  pinMode(Box1Unlocked, OUTPUT);
  pinMode(Box1UnlockedRead, INPUT);
  pinMode(Box1Locked, OUTPUT);
  pinMode(Box1LockedRead, INPUT);
  pinMode(Switch1, OUTPUT);
  pinMode(Switch1Read, INPUT);
  pinMode(Latch1, OUTPUT);

  //Assign Launcher 2 Pinmodes
  pinMode(Box2Unlocked, OUTPUT);
  pinMode(Box2UnlockedRead, INPUT);
  pinMode(Box2Locked, OUTPUT);
  pinMode(Box2LockedRead, INPUT);
  pinMode(Switch2, OUTPUT);
  pinMode(Switch2Read, INPUT);
  pinMode(Latch2, OUTPUT);

  //Assign Launcher 3 Pinmodes
  pinMode(Box3Unlocked, OUTPUT);
  pinMode(Box3UnlockedRead, INPUT);
  pinMode(Box3Locked, OUTPUT);
  pinMode(Box3LockedRead, INPUT);
  pinMode(Switch3, OUTPUT);
  pinMode(Switch3Read, INPUT);
  pinMode(Latch3, OUTPUT);

  //Assign Launcher 4 Pinmodes
  pinMode(Box4Unlocked, OUTPUT);
  pinMode(Box4UnlockedRead, INPUT);
  pinMode(Box4Locked, OUTPUT);
  pinMode(Box4LockedRead, INPUT);
  pinMode(Switch4, OUTPUT);
  pinMode(Switch4Read, INPUT);
  pinMode(Latch4, OUTPUT);

  //Assign Launcher 1 LED and Servo Positions
  digitalWrite(Box1Locked, HIGH);
  digitalWrite(Switch1, HIGH);
  Box1Lock1.write(ServoStart);
  Box1Lock2.write(ServoStart);

  //Assign Launcher 2 LED and Servo Positions
  digitalWrite(Box2Locked, HIGH);
  digitalWrite(Switch2, HIGH);
  Box2Lock1.write(ServoStart);
  Box2Lock2.write(ServoStart);

  //Assign Launcher 3 LED and Servo Positions
  digitalWrite(Box3Locked, HIGH);
  digitalWrite(Switch3, HIGH);
  Box3Lock1.write(ServoStart);
  Box3Lock2.write(ServoStart);

  //Assign Launcher 4 LED and Servo Positions
  digitalWrite(Box4Locked, HIGH);
  digitalWrite(Switch4, HIGH);
  Box4Lock1.write(ServoStart);
  Box4Lock2.write(ServoStart);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  //Code for LCD Display 
  float sensorValue = analogRead(sensorPin);
  float sensorVoltage = sensorValue * 5/1024;
  float batteryVoltage = sensorVoltage * 60.63/10.01;
  lcd.setCursor(0,0); 
  lcd.print(batteryVoltage);
  lcd.print(" V / ");
  lcd.print(batteryThreshold);
  lcd.print(" V ");
  lcd.setCursor(0,1);
  //lcd.print();
  //lcd.print();
  lcd.print("Percentage: "); 
  int batteryPercent = (batteryVoltage-batteryThreshold)/(batteryMaximum-batteryThreshold)*100;
  lcd.print(batteryPercent);
  lcd.print("%");



  //Code for launcher 1
  if(digitalRead(Switch1Read) == HIGH) {
    Box1Lock1.write(ServoEnd);
    Box1Lock2.write(ServoEnd);
    digitalWrite(Latch1, HIGH);
    digitalWrite(Box1Locked, LOW);
    delay(200);
    digitalWrite(Box1Unlocked, HIGH);
    
  } else {
    digitalWrite(Box1Unlocked, LOW);
    delay(200);
    digitalWrite(Box1Locked, HIGH);
    digitalWrite(Latch1, LOW);
    Box1Lock1.write(ServoStart);
    Box1Lock2.write(ServoStart);
  }
  
  //code for launcher 2
  if(digitalRead(Switch2Read) == HIGH) {
    Box2Lock1.write(ServoEnd);
    Box2Lock2.write(ServoEnd);
    digitalWrite(Latch2, HIGH);
    digitalWrite(Box2Locked, LOW);
    delay(200);
    digitalWrite(Box2Unlocked, HIGH);
    
  } else {
    digitalWrite(Box2Unlocked, LOW);
    delay(200);
    digitalWrite(Box2Locked, HIGH);
    digitalWrite(Latch2, LOW);
    Box2Lock1.write(ServoStart);
    Box2Lock2.write(ServoStart);
  }

  //code for launcher 3
  if(digitalRead(Switch3Read) == HIGH) {
    Box3Lock1.write(ServoEnd);
    Box3Lock2.write(ServoEnd);
    digitalWrite(Latch3, HIGH);
    digitalWrite(Box3Locked, LOW);
    delay(200);
    digitalWrite(Box3Unlocked, HIGH);
    
  } else {
    digitalWrite(Box3Unlocked, LOW);
    delay(200);
    digitalWrite(Box3Locked, HIGH);
    digitalWrite(Latch3, LOW);
    Box3Lock1.write(ServoStart);
    Box3Lock2.write(ServoStart);
  }

  //code for launcher 4
  if(digitalRead(Switch4Read) == HIGH) {
    Box4Lock1.write(ServoEnd);
    Box4Lock2.write(ServoEnd);
    digitalWrite(Latch4, HIGH);
    digitalWrite(Box4Locked, LOW);
    delay(200);
    digitalWrite(Box4Unlocked, HIGH);
    
  } else {
    digitalWrite(Box4Unlocked, LOW);
    delay(200);
    digitalWrite(Box4Locked, HIGH);
    digitalWrite(Latch4, LOW);
    Box4Lock1.write(ServoStart);
    Box4Lock2.write(ServoStart);
  }


    


}
