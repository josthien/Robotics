#include <Servo.h>
#include <Adafruit_RGBLCDShield.h>

Servo leftServo;
Servo rightServo;


unsigned long timeElapsed;

void setup() 
{
  Serial.begin(9600);
  leftServo.attach(2);
  rightServo.attach(3);
}

void loop() 
{
  timeElapsed = millis();

  if (timeElapsed > 10000)
  {
     moveStop();
  }

  delay(200);  
}

void moveForward()
{
  leftServo.writeMicroseconds(1600); 
  rightServo.writeMicroseconds(1400); 
}

void moveStop()
{
  leftServo.write(90); 
  rightServo.write(90); 
}

void moveRight()  // Dia 30 inches
{
  leftServo.writeMicroseconds(1600); 
  rightServo.writeMicroseconds(1422);
}

void moveLeft()  // Radius 30 inches
{
  leftServo.writeMicroseconds(1620); 
  rightServo.writeMicroseconds(1400);
}
