#include <RunningMedian.h>
#include <Servo.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

const int ShortFrontSensor = A0;
RunningMedian frontSensorSamples = RunningMedian(10);

Servo leftServo;
Servo rightServo;

unsigned long timeElapsed;
uint8_t buttons;
long last;  //last time you switched operations

float Pterm = 1;  //Kp Proportional Gain for closed loop feedback (Test values of 0.5, 1, 3, 5 and 20).
float desiredDistance = 5.0;  //r(t) = Desired distance to the goal.
float actualDistance; //y(t) = Distance from robot to goal. This is the actual distance value measured by the sensor.
float error;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float controlSignal;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

void switchState();
void moveForward();
void moveBack();
void moveStop();

int saturationFunctionRight(); 

void setup() 
{
  Serial.begin(9600);
  leftServo.attach(2);
  rightServo.attach(3);
  moveStop();
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("STOP");
  lcd.setBacklight(RED);
}

void loop() 
{ 
  actualDistance = calculateFrontDistance();
  error = desiredDistance - actualDistance;
  controlSignal = Pterm*error;
  outputControlSignalRight = saturationFunctionRight(controlSignal);
  outputControlSignalLeft = saturationFunctionLeft(controlSignal);

  leftServo.writeMicroseconds(outputControlSignalLeft);
  rightServo.writeMicroseconds(outputControlSignalRight);
  delay(10);
}


void moveForward()
{
  leftServo.writeMicroseconds(1615); 
  rightServo.writeMicroseconds(1300);
  lcd.setBacklight(GREEN); 
}

void moveBack()
{
  leftServo.writeMicroseconds(1300); 
  rightServo.writeMicroseconds(1700); 
}

void moveStop()
{
  leftServo.writeMicroseconds(1500); 
  rightServo.writeMicroseconds(1500);
  lcd.setBacklight(RED); 
}

double calculateFrontDistance()
{
  int sensorValue  = analogRead(ShortFrontSensor);
  frontSensorSamples.add(sensorValue);
  long m = frontSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);
   
  lcd.setCursor(0,0);
  lcd.print("F:");
  lcd.setCursor(2,0);
  if (distance >= 2.0 && distance <= 10.0)
    lcd.print(distance);
  else
    lcd.print("XXXX");

  return distance;
}

int saturationFunctionRight(float ctrlSignal)
{
  int outputCtrlSignal;
  
  if (ctrlSignal >= -10.00 && ctrlSignal <= 10.00)
  {
    outputCtrlSignal = 20*ctrlSignal + 1500;
  }
  else if (ctrlSignal > 10.00)
  {
    outputCtrlSignal = 1700;
  }
  else if (ctrlSignal < -10.00)
  {
    outputCtrlSignal = 1300;   
  }

  return outputCtrlSignal;
}

int saturationFunctionLeft(float ctrlSignal)
{
  int outputCtrlSignal;
  
  if (ctrlSignal >= -10.00 && ctrlSignal <= 10.00)
  {
    outputCtrlSignal = -20*ctrlSignal + 1500;
  }
  else if (ctrlSignal > 10.00)
  {
    outputCtrlSignal = 1300;
  }
  else if (ctrlSignal < -10.00)
  {
    outputCtrlSignal = 1700;   
  }

  return outputCtrlSignal;
}
