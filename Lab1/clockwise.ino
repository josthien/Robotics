#include <Servo.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

Servo leftServo;
Servo rightServo;

unsigned long timeElapsed;
uint8_t buttons;
long last;  //last time you switched operations
int stepCount = 0;  //to keep track of steps. total of 11 steps from 0 to 10. 
int state = 0; //to choose between different states. 0 is stop. 1 is forward. 2 is turning. 
int mode = 0; //to choose between left and right turn modes. 1 is left. 2 is right.
int turning = 0; //to keep track of whether the bot is turning or not. 1 is No. 2 is Yes.
bool action = false;  //whether the bot is moving/needs to move or not 
unsigned long distanceThreshold;  //distance threshold for moving forward and choosing between 15 inches and 30 inches. 
unsigned long turnThreshold = 640;  //distance threshold for turning 90 degrees right or left.

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
void moveLeft();
void moveRight();
void turnRight();
void turnLeft();
void updateDistanceThreshold();

void setup() 
{
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
    case 0: // for stop state
        action = false; // not moving
        turning = 0; // not turning
        moveStop();
      break;
    case 1: // for forward state
      if (timeElapsed - last < distanceThreshold)  
      {
        turning = 1;
        action = true; // moving
        moveForward(); // move forward
      }
      else
      {
        turning = 2; // turning
        action = false; // not moving
      }
      break;
    case 2: // for turning state
      if (timeElapsed - last < turnThreshold)  
      {
        turning = 2; // turning
        action = true; // moving
        if (mode == 1)
          turnLeft();
        else
          turnRight();
      }
      else
      {
        turning = 1;
        action = false;
      }
      break;
  }

  delay(10);
}

void moveForward()
{
  leftServo.writeMicroseconds(1605); 
  rightServo.writeMicroseconds(1300); 
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
}

void moveRight()  // Dia 30 inches
{
  leftServo.writeMicroseconds(1604); 
  rightServo.writeMicroseconds(1450);
}

void moveLeft()  // Dia 30 inches
{
  leftServo.writeMicroseconds(1551); 
  rightServo.writeMicroseconds(1400);
}

void turnRight()  //90 degree right turn
{
  leftServo.writeMicroseconds(1700); 
  rightServo.writeMicroseconds(1700);
}

void turnLeft()  //90 degree left turn
{
  leftServo.writeMicroseconds(1300); 
  rightServo.writeMicroseconds(1300);
}

void updateDistanceThreshold()
{
  if (stepCount == 1)
    distanceThreshold = 2700; //for a distance of 15 inches
  else if (stepCount == 9)
    distanceThreshold = 2700; //for a distance of 15 inches    
  else 
    distanceThreshold = 5400; //for a distance of 30 inches
}

void switchState()
{
  if (stepCount == 9 && action == false)  // Stop state transition
  {  
    state = 0; //for stop state
    last = timeElapsed;
    mode = 0;
    stepCount =  0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("STOP");
    lcd.setBacklight(RED);
  } 
  else if (((buttons & BUTTON_LEFT) && action == false) || (turning == 1 && action == false && mode == 1))  //Forward state transition with mode left
  {
    state = 1;  //setting state as forward
    mode = 1; //setting mode left
    last = timeElapsed;
    stepCount =  stepCount + 1;
    updateDistanceThreshold();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FORWARD");
    lcd.setBacklight(GREEN);
  }
  else if (((buttons & BUTTON_RIGHT) && action == false) || (turning == 1 && action == false && mode == 2)) //Forward state transition with mode right
  {
    state = 1; //setting state as forward
    mode = 2; //for mode right
    last = timeElapsed;
    stepCount =  stepCount + 1;
    updateDistanceThreshold();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FORWARD");
    lcd.setBacklight(GREEN);
    
  }
  else if (turning == 2 && action == false) //Turning state transition
  {
    state = 2; //for turn state
    last = timeElapsed;
    stepCount =  stepCount + 1;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TURN");
    lcd.setBacklight(BLUE);
  }
}
