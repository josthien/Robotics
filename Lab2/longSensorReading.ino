#include <RunningMedian.h>
#include <Servo.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

const int ShortFrontSensor = A0;
const int ShortLeftSensor = A1; 
const int ShortRightSensor = A2;

RunningMedian frontSensorSamples = RunningMedian(10);
RunningMedian leftSensorSamples = RunningMedian(10);
RunningMedian rightSensorSamples = RunningMedian(5);

Servo leftServo;
Servo rightServo;

unsigned long timeElapsed;
uint8_t buttons;
long last;  //last time you switched operations
int state = 0;  //6 states from 0 to 5
unsigned long rightTurnThreshold = 2200;
unsigned long leftTurnThreshold = 1800;
unsigned long distanceThreshold = 2000;  //distance threshold for moving forward 

float Pterm = 1.0;  //Kp Proportional Gain for closed loop feedback (Test values of 0.5, 1, 3, 5 and 20).
float desiredDistance = 5.0;  //r(t) = Desired distance to the goal.
float actualDistance; //y(t) = Distance from robot to goal. This is the actual distance value measured by the sensor.
float error;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float controlSignal;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

float Pterm2 = 3.0;  //Kp Proportional Gain for closed loop feedback (Test values of 0.5, 1, 3, 5 and 20).
float Iterm2 = 1.0;   //Ki
double desiredDistance2 = 5.0;  //r(t) = Desired distance to the goal.
double actualDistance2; //y(t) = Distance from robot to goal. This is the actual distance value measured by the sensor.
float error2;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float error2Sum = 0.0;
double controlSignal2;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight2;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft2;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

bool turning = false;
bool forward = false;
bool action = false;

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

void switchState();
void moveForward();
void moveStop();
void moveRight();
void turnLeft();
int saturationFunctionRight();
int saturationFunctionLeft(); 
float saturationFunctionDistance();


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
  timeElapsed = millis();
  buttons = lcd.readButtons();

  switchState();

  switch(state)
  {
    case 0: //Stop
      moveStop();
      break;
    case 1: //Move Right
      if (timeElapsed - last < rightTurnThreshold)
      {
        moveRight();
      }
      else
      {
        forward = true;
      }

      break;
    case 2: //Forward with proportional closed loop
      actualDistance = saturationFunctionDistance(calculateFrontDistance());
      actualDistance2 = saturationFunctionDistance(calculateRightDistance());
      
      error = desiredDistance - actualDistance;
      controlSignal = Pterm*error;
    
      error2 = desiredDistance2 - actualDistance2;
      controlSignal2 = Pterm2*error2;
      
      outputControlSignalRight = saturationFunctionRight(controlSignal);
      outputControlSignalLeft = saturationFunctionLeft(controlSignal);
    
      leftServo.writeMicroseconds(outputControlSignalLeft-controlSignal2);
      rightServo.writeMicroseconds(outputControlSignalRight-controlSignal2);

      break;
    case 3: //Forward open loop
      if (timeElapsed - last < distanceThreshold)
      {
        moveForward();
      }
      else
      {
        forward = false;
        turning = false;
        action = false;
        while (calculateRightDistance() >= 8.0)
        {}
      }
      break;
    case 4: //Turn Left
      if (timeElapsed - last < leftTurnThreshold)
      {
        turnLeft();
      }
      else
      {
        forward = true;
        action = false;
        turning = true;
        while (calculateFrontDistance() <= 7.0)
        {}
      }
      break;
      
  }
}

void moveForward()
{
  leftServo.writeMicroseconds(1520); 
  rightServo.writeMicroseconds(1480); 
}

void moveStop()
{
  leftServo.writeMicroseconds(1500); 
  rightServo.writeMicroseconds(1500);
}

void moveRight()  // Turn corner
{
  leftServo.writeMicroseconds(1600); //1600 
  rightServo.writeMicroseconds(1475); //1458
}

void turnRight()  //90 degree right turn
{
  leftServo.writeMicroseconds(1550); 
  rightServo.writeMicroseconds(1550);
}

void turnLeft()  //90 degree left turn
{
  leftServo.writeMicroseconds(1470); 
  rightServo.writeMicroseconds(1470);
}

void switchState()
{
  if ((turning == false) && (forward == false) && (calculateRightDistance() <= 10.0))
  {
    state = 2;  //Forward with closed loop proportional control
    last = timeElapsed;

    if ((turning == false) && (forward == false) && (calculateFrontDistance() <= 7.0))
    {
      state = 4;  //Turn left
      turning = true;
      last = timeElapsed;
    }
  }
  
  else if ((turning == false) && (forward == false) && (calculateRightDistance() > 10.0))
  {
    state = 1;
    turning = true;
    last = timeElapsed;
  }

  else if ((forward == true) && (action == false))
  {
    state = 3;
    action = true;
    last = timeElapsed;
  } 
}

double calculateRightDistance()
{
  //right sensor
  int sensorValue  = analogRead(ShortRightSensor);
  rightSensorSamples.add(sensorValue);
  long m = rightSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);
  
  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.setCursor(2,1);
  
  if (distance >= 2.0 && distance <= 10.0)
    lcd.print(distance);
  else
    lcd.print("XXXX");

    return distance;
}

double calculateFrontDistance()
{
  int sensorValue  = analogRead(ShortFrontSensor);
  frontSensorSamples.add(sensorValue);
  long m = frontSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);

  return distance; 
}

int saturationFunctionRight(float ctrlSignal)
{
  int outputCtrlSignal;
  
  if (ctrlSignal >= -20.00 && ctrlSignal <= 20.00)
  {
    outputCtrlSignal = 10*ctrlSignal + 1500;
  }
  else if (ctrlSignal > 20.00)
  {
    outputCtrlSignal = 1700;
  }
  else if (ctrlSignal < -20.00)
  {
    outputCtrlSignal = 1300;   
  }

  return outputCtrlSignal;
}

int saturationFunctionLeft(float ctrlSignal)
{
  int outputCtrlSignal;
  
  if (ctrlSignal >= -20.00 && ctrlSignal <= 20.00)
  {
    outputCtrlSignal = -10*ctrlSignal + 1500;
  }
  else if (ctrlSignal > 20.00)
  {
    outputCtrlSignal = 1300;
  }
  else if (ctrlSignal < -20.00)
  {
    outputCtrlSignal = 1700;   
  }

  return outputCtrlSignal;
}

float saturationFunctionDistance(float goaldistance)
{
  if (goaldistance <= 10.0)
  {
    return goaldistance;
  }
  else
  {
    return 10.0;
  }
}
