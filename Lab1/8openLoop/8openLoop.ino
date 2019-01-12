/* Description
  The robot moves in a "8" path. Each circle part of the "8" path has the radius of 30 inches
*/
#include <Servo.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

Servo leftServo;
Servo rightServo;

unsigned long timeElapsed;
uint8_t buttons;
long last;  //last time you switched operations
int stepCount = 0;  //to keep track of steps. total of 7 steps from 1 to 7. 
int state = 0; //to choose between different states. 0 is stop. 1 is turning. 2 is forward.
int mode = 0; //to keep track of whether the bot is turning left or right. 1 is left. 2 is right. 
bool action = false;  //whether the bot is moving/needs to move or not 
bool turning = false; //to keep track of whether the bot is turning or not.
bool forward = false; //to keep track of whether the bot is moving forward or not
unsigned long distanceThreshold = 6000;  //distance threshold for moving forward
unsigned long turnThreshold;  //distance threshold for moving right or left in a circle

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
void moveLeft();
void moveRight();
void turnRight();
void turnLeft();
void updateTurningThreshold();

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
        action = false;
        turning = false;
        forward = false;
        moveStop();
      break; 
    case 1: // for turning
      if (timeElapsed - last < turnThreshold)  
      {
        turning = true;
        action = true;
        forward = false;
        if (mode == 1)
          moveLeft();
        else if (mode == 2)
          moveRight();
      }
      else
      {
        turning = false;
        action = false;
        forward = true;
      }
      break;
    case 2: // for forward state
      if (timeElapsed - last < distanceThreshold)  
      {
        turning = false;
        action = true;
        forward = true;
        moveForward();
      }
      else
      {
        turning = true;
        action = false;
        forward = false;
      }
      break;
  }
  delay(10);
}

void moveForward()
{
  leftServo.writeMicroseconds(1615); 
  rightServo.writeMicroseconds(1300); 
}

void moveStop()
{
  leftServo.writeMicroseconds(1500); 
  rightServo.writeMicroseconds(1500); 
}

void moveRight()  // Dia 30 inches
{
  leftServo.writeMicroseconds(1600); //1600 
  rightServo.writeMicroseconds(1457); //1458
}

void moveLeft()  // Dia 30 inches
{
  leftServo.writeMicroseconds(1545);//1551 
  rightServo.writeMicroseconds(1400);//1400
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

void updateTurningThreshold()
{
  if (mode == 2)
    turnThreshold = 8000; //for a turn angle of 120 degrees.
  else if (mode == 1)
    turnThreshold = 15500; //for a turn angle of 240 degrees. 
}

void switchState()
{
  if (stepCount == 5 && action == false)  // Stop state transition
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
  else if (((buttons & BUTTON_UP) && action == false) || (action == false && mode == 1 && turning == true && forward == false))  //Right move transition with angle 120 degrees.
  {
    state = 1;  //setting state as turning
    mode = 2; //setting circle direction as right
    last = timeElapsed;
    stepCount =  stepCount + 1;
    updateTurningThreshold();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TURN");
    lcd.setBacklight(GREEN);
  }
  else if (action == false && mode == 2 && turning == true && forward == false) //Left move transition with angle 240 degrees.
  {
    state = 1; //setting state as turning
    mode = 1; //setting circle direction as left
    last = timeElapsed;
    stepCount =  stepCount + 1;
    updateTurningThreshold();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TURN");
    lcd.setBacklight(GREEN);
    
  }
  else if (action == false && forward == true && turning == false) //Forward state transition
  {
    state = 2; //setting state as forward
    last = timeElapsed;
    stepCount =  stepCount + 1;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FORWARD");
    lcd.setBacklight(BLUE);
  }
}
