#include <LiquidCrystal_I2C.h>

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Define variables for the launcher
int LatchPin = 46; //Assigns pin that controls the latch
int ServoClosed = 135; //Position the Servo starts in 
int ServoOpen = 85; //Position the Servo finishes at
int SwitchPin = 35; //Pin that the On-Switch is connected to

//Setup the Servo motors
Servo Lock1; //Define two servos for the locks
Servo Lock2;

//Setup the LCD Display
int SensorPin = A8; //Pin that reads battery voltage
float BatteryThreshold = 11.2; //SET THIS BASED ON BATTERY TYPE (8 AA)
float BatteryMaximum = 16.5; //SET THIS BASED ON BATTERY (8 AA)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // put your setup code here, to run once:
  Lock1.attach(48); //Assign lock 1 to pin 48
  Lock2.attach(49); //Assign lock 2 to pin 49
  pinMode(LatchPin, OUTPUT); //Assign the Latch control as an output
  pinMode(SwitchPin, INPUT); //Assign the switch as an input
  pinMode(SensorPin, INPUT); //Assign Sensor voltage as an input
 
  Lock1.write(ServoClosed);  //Assigns servos to the locked position
  Lock2.write(ServoClosed);  //Assigns servos to the locked position
 
  //Setup LCD Basic functions for the LCD
  lcd.begin(); 
  lcd.clear();
  lcd.print("ON");
  lcd.backlight();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Code for the LCD Display
  float SensorValue = analogRead(SensorPin); //Read the input voltage value
  float SensorVoltage = SensorValue * 5/1024; //This is the breakdown of the voltage steps in Arduino
  float BatteryVoltage = SensorVoltage* 60.93/10.01; //Multiply by the voltage divider (R1+R2)/R1
  int BatteryPercent = (BatteryVoltage-BatteryThreshold)/(BatteryMaximum-BatteryThreshold)*100;
  lcd.begin();
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print(BatteryVoltage);//Prints the voltage left in the battery
  lcd.print("V / ");
  lcd.print(BatteryThreshold);
  lcd.print(" V ");
  lcd.setCursor(0,1); //Moves cursor to second line
  lcd.print("Percentage: ");
  lcd.print(BatteryPercent);
  lcd.print("%");

  //Check the Switch and open the locks/latch
  int SwitchRead = digitalRead(SwitchPin);
  if(SwitchRead == HIGH){
    Lock1.write(ServoOpen); //Sets servo to unlocked position
    Lock2.write(ServoOpen); //Sets servo to unlocked position
    digitalWrite(LatchPin, HIGH); //Unlock the Latch

    //Print that the launcher has unlocked on the LCD
    lcd.clear(); //Clear the LCD Display
    lcd.setCursor(0,0); //Set cursor to first line
    lcd.print("Launcher 1 Open");
    lcd.setCursor(0,1);
    lcd.print("Ready to Launch");
    delay(5000);
  }
  else{
    Lock1.write(ServoClosed); //Close the lock 1
    Lock2.write(ServoClosed); //Close the lock 2
    digitalWrite(LatchPin, LOW); // Lock the latch

    //Print that the launcher is locked. This can be added, but not necessary
    //lcd.clear();
    //lcd.setCursor(0,0);
    //lcd.print("Launcher 1");
    //lcd.setCursor(0,1);
    //lcd.print("Closed");
  }
  
}
