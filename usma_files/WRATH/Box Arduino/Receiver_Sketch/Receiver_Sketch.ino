//READ ME
//This is preliminary testing sketch for using the RF Reciever. Worked, but had some difficulty 
//and bugs in the connection. Recommend purchasing a higher quality reciever, or working
//with I2C protocol on arduino

int receiverPin = 8;
int ledPin = 13;

void setup() {
  // put your setup code here, to run once:

pinMode(receiverPin, INPUT);
pinMode(ledPin, OUTPUT);
}

void loop() {
int fobRead = digitalRead(receiverPin);
if (fobRead == HIGH){
  digitalWrite(ledPin, HIGH);
}
else{
  digitalWrite(ledPin, LOW);
}
  // put your main code here, to run repeatedly:

}
