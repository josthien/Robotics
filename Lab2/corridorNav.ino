#include <RunningMedian.h>
#include <Servo.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

const int ShortFrontSensor = A0;
const int ShortLeftSensor = A1; 
const int ShortRightSensor = A2;

RunningMedian frontSensorSamples = RunningMedian(10);
RunningMedian leftSensorSamples = RunningMedian(5);
RunningMedian rightSensorSamples = RunningMedian(5);

Servo leftServo;
Servo rightServo;

unsigned long timeElapsed;
uint8_t buttons;
long last;  //last time you switched operations
int state = 0; 
unsigned long rightTurnThreshold = 2000;
unsigned long distanceThreshold = 2000;  //distance threshold for moving forward
int redColorCounter = 0;  //Counter to keep track of how many times robot crosses over red floor marker
int blueColorCounter = 0;  //Counter to keep track of how many times robot crosses over blue floor marker
int brownColorCounter = 0;  //Counter to keep track of how many times robot crosses over brown floor marker

float Pterm = 1.0;  //Kp Proportional Gain for closed loop feedback (Test values of 0.5, 1, 3, 5 and 20).
float desiredDistance = 5.0;  //r(t) = Desired distance to the goal.
float actualDistance; //y(t) = Distance from robot to goal. This is the actual distance value measured by the sensor.
float error;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float controlSignal;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

float Pterm2 = 3.0;  //Kp Proportional Gain for closed loop feedback
float Iterm2 = 1.0;   //Ki
double desiredDistance2 = 3.0;  //r(t) = Desired distance to right wall
double actualDistance2; //y(t) = Actual distance to right wall
float error2;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float error2Sum = 0.0;
double controlSignal2;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight2;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft2;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

float Pterm3 = 3.0;  //Kp Proportional Gain for closed loop feedback
float Iterm3 = 1.0;   //Ki
double desiredDistance3 = 3.0;  //r(t) = Desired distance to left wall
double actualDistance3; //y(t) = Actual distance to left wall
float error3;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float error3Sum = 0.0;
double controlSignal3;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight3;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft3;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

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

// COLOR SENSOR
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

int fRed;
int fGreen;
int fBlue;

void switchState();
void moveForward();
void moveStop();
void moveRight();
void turnRight();
int saturationFunctionRight();
int saturationFunctionLeft(); 
float saturationFunctionDistance();


void setup() {
  Serial.begin(9600);
  leftServo.attach(2);
  rightServo.attach(3);
  moveStop();
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.setBacklight(WHITE);

  //SET COLOR SENSOR
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

}

void loop() 
{
  timeElapsed = millis();
  buttons = lcd.readButtons();

  checkColor();

  switchColor();
  
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
      actualDistance3 = saturationFunctionDistance(calculateLeftDistance());
      
      error = desiredDistance - actualDistance;
      controlSignal = Pterm*error;
    
      error2 = desiredDistance2 - actualDistance2;
      controlSignal2 = Pterm2*error2;

      error3 = desiredDistance3 - actualDistance3;
      controlSignal3 = Pterm3*error3;
      
      outputControlSignalRight = saturationFunctionRight(controlSignal);
      outputControlSignalLeft = saturationFunctionLeft(controlSignal);
    
      leftServo.writeMicroseconds(outputControlSignalLeft-controlSignal2+controlSignal3);
      rightServo.writeMicroseconds(outputControlSignalRight-controlSignal2+controlSignal3);

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
        while (calculateRightDistance() >= 6.0)
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
  rightServo.writeMicroseconds(1478); //1458
}

void turnRight()  //90 degree right turn
{
  leftServo.writeMicroseconds(1530); 
  rightServo.writeMicroseconds(1530);
}

void switchState()
{
  if (redColorCounter == 1 && blueColorCounter == 1 && brownColorCounter == 0)
  {
    state = 0;
  }
  else if ((turning == false) && (forward == false) && (calculateRightDistance() <= 10.0))
  {
    state = 2;  //Forward with closed loop proportional control
    last = timeElapsed;
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
   
  lcd.setCursor(7,1);
  lcd.print("F:");
  lcd.setCursor(9,1);
  if (distance >= 2.0 && distance <= 10.0)
    lcd.print(distance);
  else
    lcd.print("XXXX");

    return distance; 

}

double calculateLeftDistance()
{
  int sensorValue  = analogRead(ShortLeftSensor);
  leftSensorSamples.add(sensorValue);
  long m = leftSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);
   
  lcd.setCursor(0,0);
  lcd.print("L:");
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

void checkColor()
{
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  fRed = pulseIn(sensorOut, LOW);

  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  fGreen = pulseIn(sensorOut, LOW);

  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  fBlue = pulseIn(sensorOut, LOW);
}

bool isRed()
{
  if ((fRed > 10 && fRed < 15) && (fGreen > 55 && fGreen < 65) && (fBlue > 35 && fBlue < 45))
  {
    return true;
  }
  else
  {
    return false;
  } 
}

bool isBlue()
{
  if ((fRed > 85 && fRed < 105) && (fGreen > 95 && fGreen < 115) && (fBlue > 50 && fBlue < 65))
  {
    return true;
  }
  else
  {
    return false;
  } 
}

bool isBrown()
{
  if ((fRed > 40 && fRed < 60) && (fGreen > 80 && fGreen < 105) && (fBlue > 80 && fBlue < 105))
  {
    return true;
  }
  else
  {
    return false;
  } 
}

void switchColor()
{
  if (isRed())
  {
    redColorCounter = 1;
    brownColorCounter = 0;
    lcd.setBacklight(RED);
  }

  if (isBlue())
  {
    blueColorCounter = 1;
    lcd.setBacklight(BLUE);
  }

  if (isBrown())
  {
    brownColorCounter = 1;
    lcd.setBacklight(YELLOW);
  }
}
