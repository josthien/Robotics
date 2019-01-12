
//Including libraries
#include <RunningMedian.h>
#include <Servo.h>
#include <Adafruit_RGBLCDShield.h>
#include <QueueArray.h>

//Robot and World variables declaration

//Headings declarations
typedef enum {WEST, NORTH, SOUTH, EAST} heading;
#define WEST 0
#define NORTH 1
#define SOUTH 2
#define EAST 3

//World map declarations
typedef enum {WALL, FREE, VISITED, FIXEDWALL, NOTKNOWN} grid;
#define WALL 'X'
#define FREE '0'
#define VISITED '*'
#define FIXEDWALL '|'
#define NOTKNOWN 'U'

//Defining structure type for world map
typedef struct {
  grid square[9][9];
  int targetX, targetY; //Move these variables to a separate struct. TECHNICAL DEBT!!!
  int targetGridNumber;
  heading targetHeading;
  int goalGrid;
  int startGrid;
  int startHeading;
  int maptype;
} worldMap;

//Defining structure type for robot
typedef struct {
    int x, y; //Position x and y
    int currentGrid;
    heading theta;  //Angular heading
    grid sensorLeft, sensorFront, sensorRight;  //distance sensors
    grid north, south, east, west;  //Robot global compass bearings
} robotPosition;

worldMap world;   //Creating world struct instance
robotPosition robot;  //Creating robot struct instance

//Variable declarations

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

const int ShortFrontSensor = A0;
const int ShortLeftSensor = A1; 
const int ShortRightSensor = A2;

RunningMedian frontSensorSamples = RunningMedian(50);
RunningMedian leftSensorSamples = RunningMedian(50);
RunningMedian rightSensorSamples = RunningMedian(50);

Servo leftServo;
Servo rightServo;

int iterationCounter = 0;
unsigned long timeElapsed;
unsigned long stepsMoved;
uint8_t buttons;
long last;  //last time you switched operations
long lastSteps;  //number of steps moved last before switching state
long colorLast; //color last time timer variable
int state = 0;  //
unsigned long rightTurnThreshold = 1450;
unsigned long leftTurnThreshold = 1650;
unsigned long uTurnThreshold = 2400;
unsigned long distanceThreshold = 5800;  //distance threshold for moving forward one box on the grid
//unsigned long stepsThreshold = 150;  //steps threshold for moving forward one box on the grid
unsigned long colorThreshold = 50;  //time threshold for color flashing

//Front proportional closed loop feedback variables
float Pterm = 1.0;  //Kp Proportional Gain for closed loop feedback (Test values of 0.5, 1, 3, 5 and 20).
float desiredDistance = 5.0;  //r(t) = Desired distance to the goal.
float actualDistance; //y(t) = Distance from robot to goal. This is the actual distance value measured by the sensor.
float error;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float controlSignal;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

//Right proportional closed loop feedback variables
float Pterm2 = 3.0;  //Kp Proportional Gain for closed loop feedback
double desiredDistance2 = 8.0;  //r(t) = Desired distance to right wall
double actualDistance2; //y(t) = Actual distance to right wall
float error2;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float error2Sum = 0.0;
double controlSignal2;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight2;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft2;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

//Left proportional closed loop feedback variables
float Pterm3 = 2.0;  //Kp Proportional Gain for right closed loop feedback
double desiredDistance3 = 6.0;  //r(t) = Desired distance to left wall
double actualDistance3; //y(t) = Actual distance to left wall
float error3;  //e(t) = r(t) - y(t); //desiredDistance minus actualDistance.
float error3Sum = 0.0;
double controlSignal3;  //u(t) = Kp*e(t) (control signal corresponding to robot velocity)
int outputControlSignalRight3;  //ur(t) (right motor control signal corresponding to saturated robot velocity)
int outputControlSignalLeft3;  //ur(t) (left motor control signal corresponding to saturated robot velocity)

bool forward = false;
bool left = false;
bool right = false;
bool uturn = false;
bool action = false;
bool mapped = false;
bool visited = false;
bool followed = false;
bool planned = false;
bool robotsetup = false;

//ENCODERS
int lEncPin = 10;
int rEncPin = 11;
int rEncCount = 0;
int lEncCount = 0;
int lEncLast = 0;
int rEncLast = 0;

//Queue Declarations
QueueArray <int> shortestPath;
QueueArray <int> floodFillAdjacenctGrids;

//PROGRAM STATES
#define NUM_STATES 5

#define CHOOSE_PROGRAM 0
#define SETUP 1
#define MAPPING 2
#define PATH_PLANNING 3
#define PATH_FOLLOWING 4

#define NUM_SELECTION 5

#define START 0
#define END 1
#define HEADING 2
#define MAP 3
#define EXIT 4

const char* names[] = {"CHOOSE PROGRAM","SETUP ROBOT","MAPPING MAZE", "PATH PLANNING", "PATH FOLLOWING"};
const char* selectnames[] = {"Starting Locat.","Ending Locat.","Orientation", "Map Type", "Exit"};
const char* orientation[] = {"West","North","South", "East"};
const char* mapnames[] = {"Map 1","Map 2","Map 3", "Unknown Map"};

int stateLCD = CHOOSE_PROGRAM;
int program = SETUP;
int selection = START;

//FUNCTION DECLARATIONS
void pathFollowing();
void pathPlanning();
void mappingState();
void chooseProgram();
void setupRobot();


void setup() {
  
  Serial.begin(9600);
  leftServo.attach(2);
  rightServo.attach(3);
  moveStop();
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(WHITE);

  //SET ENCODERS
  pinMode(lEncPin, INPUT_PULLUP);
  pinMode(rEncPin, INPUT_PULLUP);

  world.startGrid = 1;
  world.goalGrid = 1;
}

void loop() {

  buttons = lcd.readButtons();
  
    switch(stateLCD){
    case CHOOSE_PROGRAM:
      chooseProgram();
      break;
    case SETUP:
      setupRobot();
      break;
    case MAPPING:
      mappingState();
      break;
    case PATH_PLANNING:
      pathPlanning();
      break; 
    case PATH_FOLLOWING:
      pathFollowing();
      break;  
      
  }
}

void chooseProgram(){
  static bool titleShown = false;
  robotsetup = false;

  if(!titleShown){
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print(names[program]);
    lcd.setBacklight(GREEN);
    titleShown = true;
  }

  moveStop();
  lcd.setBacklight(GREEN);

  if (  buttons & BUTTON_SELECT ){
    stateLCD = program;
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print(names[program]);

    titleShown = false;
  } else if (  buttons & BUTTON_LEFT |  buttons & BUTTON_DOWN) {
    program = 1 + (program % (NUM_STATES-1));

    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print(names[program]);

    
  } else if ( buttons & BUTTON_RIGHT | buttons & BUTTON_UP) {
    program = program == 1 ? NUM_STATES-1 : program - 1;
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print(names[program]);
  }

  delay(100);
}

void setupRobot(){
  if((buttons & BUTTON_SELECT) && selection == EXIT){
    stateLCD = CHOOSE_PROGRAM;
    program = SETUP;
  }
  if (mapped == false){
    initializeWorldMap();
    initializeRobot();
  }

  if(robotsetup == false){

    robotsetup = true;
    lcd.clear();
    lcd.setBacklight(GREEN);
    lcd.setCursor(1,0);
    lcd.print(selectnames[selection]);
    lcd.setCursor(1,1);
    switch(selection){
      case START:
        lcd.print(world.startGrid);
        break;
      case END:
        lcd.print(world.goalGrid);
        break;
      case HEADING:
        lcd.print(orientation[world.startHeading]);
        break;
      case MAP:
        lcd.print(mapnames[world.maptype]);
        break;
    }
  }

  if (buttons & BUTTON_RIGHT) {
    selection++;
    if (selection > (NUM_SELECTION-1)){
      selection = 0;
    }
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print(selectnames[selection]);
    lcd.setCursor(1,1);
    switch(selection){
      case START:
        lcd.print(world.startGrid);
        break;
      case END:
        lcd.print(world.goalGrid);  
        break;
      case HEADING:
        lcd.print(orientation[world.startHeading]);
        break;
      case MAP:
        lcd.print(mapnames[world.maptype]);
        break;
    }
  }
  else if (buttons & BUTTON_LEFT){
    selection--;
    if (selection < 0){
      selection = (NUM_SELECTION-1);
    }
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print(selectnames[selection]);
    lcd.setCursor(1,1);
    switch(selection){
      case START:
        lcd.print(world.startGrid);
        break;
      case END:
        lcd.print(world.goalGrid);
        break;
      case HEADING:
        lcd.print(orientation[world.startHeading]);
        break;
      case MAP:
        lcd.print(mapnames[world.maptype]);
        break;
    }
  }
  else if ( buttons & BUTTON_UP) {
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print(selectnames[selection]);
    lcd.setCursor(1,1);
    switch(selection){
      case START:
        world.startGrid++;
        if (world.startGrid > 16){
          world.startGrid = 1;
        }
        lcd.print(world.startGrid);
        break;
      case END:
        world.goalGrid++;
        if (world.goalGrid > 16){
          world.goalGrid = 1;
        }
        lcd.print(world.goalGrid);
        break;
      case HEADING:
        world.startHeading++;
        if (world.startHeading > 3){
          world.startHeading = 0;
        }
        lcd.print(orientation[world.startHeading]);
        break;
      case MAP:
        world.maptype++;
        if (world.maptype > 3){
          world.maptype = 0;
        }
        lcd.print(mapnames[world.maptype]);
        break;
    }
  } 
  else if ( buttons & BUTTON_DOWN){
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print(selectnames[selection]);
    lcd.setCursor(1,1);
    switch(selection){
      case START:
        world.startGrid--;
        if (world.startGrid < 1){
          world.startGrid = 16;
        }
        lcd.print(world.startGrid);
        break;
      case END:
        world.goalGrid--;
        if (world.goalGrid < 1){
          world.goalGrid = 16;
        }
        lcd.print(world.goalGrid);
        break;
      case HEADING:
        world.startHeading--;
        if (world.startHeading < 0){
          world.startHeading = 3;
        }
        lcd.print(orientation[world.startHeading]);
        break;
      case MAP:
        world.maptype--;
        if (world.maptype < 0){
          world.maptype = 3;
        }
        lcd.print(mapnames[world.maptype]);
        break;
    }
  }
  delay(100);
}

void mappingState(){

  if(buttons & BUTTON_SELECT){
    stateLCD = CHOOSE_PROGRAM;
    program = CHOOSE_PROGRAM;
  }
  while((allSquaresVisited() == false) && visited == false){
    
    timeElapsed = millis();
    stepsMoved = encoderCount();
    buttons = lcd.readButtons();
    
    if(action == false){
      checkSensors(); //Robot checks its sensors and updates local sensor variables
      updateWorldMap();
      updateRobotCompass();
      printWorldMap();
      printLCD();
      switchState();
    }
    runState();

  }
    if(allSquaresVisited() == true && mapped == false){
    moveStop();  
    checkSensors();
    updateWorldMap();
    updateRobotCompass();
    printWorldMap();
    printLCD(); 
    mapped = true;
    visited = true;
  }
}

void pathPlanning(){
  
  if(buttons & BUTTON_SELECT){
    stateLCD = CHOOSE_PROGRAM;
    program = SETUP;
  }
  if(planned == false){
    floodFill();
    runShortestPath();
    world.targetGridNumber = shortestPath.pop();
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("Path Calculated!");
    lcd.setBacklight(RED);
    planned = true;
    initializeWorldMap();
    initializeRobot();
  }
  
}

void pathFollowing(){

  if(buttons & BUTTON_SELECT){
    stateLCD = CHOOSE_PROGRAM;
    program = SETUP;
  }
  //Code for Path follower

    while(!shortestPath.isEmpty() && world.targetGridNumber != 0){
      timeElapsed = millis();
      stepsMoved = encoderCount();
    
      if(action == false){
        if(forward == false){
          world.targetGridNumber = shortestPath.pop();
        }
        getTargetCoordinates();
        calculateTargetHeading();
        updateRobotCompass();
        printWorldMap();
        printLCD();
        switchPathFollowerState();
      }
      runState();
    }
    if(shortestPath.isEmpty() == true && followed == false){
      moveStop();
      followed = true;
    }

}

//Robot decides which state to transition to depending on sensor values
void switchState(){ 
  if(action == false){
    
    iterationCounter++;
    lastSteps = encoderCount();

    if (forward == true){
      state = 4;  //Forward move
      last = timeElapsed;
      action = true;
    }
    else if (robot.sensorLeft != WALL && robot.sensorLeft != VISITED && robot.sensorLeft != FIXEDWALL){
      state = 1;  //Left 90 degree turn
      last = timeElapsed;
      action = true;
      left = true;
    }
    else if (robot.sensorFront != WALL && robot.sensorFront != VISITED && robot.sensorFront != FIXEDWALL){
      state = 4;  //Forward move
      last = timeElapsed;
      action = true;
      forward = true;
    }
    else if(robot.sensorRight != WALL && robot.sensorRight != VISITED && robot.sensorRight != FIXEDWALL){
      state = 2;  //Right 90 degree turn
      last = timeElapsed;
      action = true;
      right = true;
    }
    else{
      if(robot.sensorLeft != WALL && robot.sensorLeft != FIXEDWALL){
        state = 1;  //Left 90 degree turn
        last = timeElapsed;
        action = true;
        left = true;
      }
      else if(robot.sensorFront != WALL && robot.sensorFront != FIXEDWALL){
        state = 4;  //Forward move
        last = timeElapsed;
        action = true;
        forward = true;
      }
      else if(robot.sensorRight != WALL && robot.sensorRight != FIXEDWALL){
        state = 2;  //Right 90 degree turn
        last = timeElapsed;
        action = true;
        right = true;
      }
      else{
        state = 3;  //90 degree uturn
        last = timeElapsed;
        action = true;
        uturn = true;
      }
    }
  }
}

void runState(){

  //Color state
  (timeElapsed - colorLast < colorThreshold) ? lcd.setCursor(0, 0) : lcd.setBacklight(WHITE);

  switch(state){
    case 0: //Stop state
        moveStop();
      break;
    case 1: //Left turn 90
      if (timeElapsed - last < leftTurnThreshold)
      {
        turnLeft();
      }
      else
      {
        action = false;
        left = false;
        moveStop();
        updateLeft();
        forward = true;
        frontSensorSamples.clear();
        leftSensorSamples.clear();
        rightSensorSamples.clear();
      }
      break;
    case 2: //Right turn 90
      if (timeElapsed - last < rightTurnThreshold)
      {
        turnRight();
      }
      else
      {
        action = false;
        right = false;
        moveStop();
        updateRight();
        forward = true;
        frontSensorSamples.clear();
        leftSensorSamples.clear();
        rightSensorSamples.clear();
      }
      break;
    case 3: //U turn 180
      if (timeElapsed - last < uTurnThreshold)
      {
        uTurn();
      }
      else
      {
        action = false;
        uturn = false;
        moveStop();
        updateUturn();
        forward = true; 
        frontSensorSamples.clear();
        leftSensorSamples.clear();
        rightSensorSamples.clear();
      }
      break;
    case 4: //Forward
      if (timeElapsed - last < distanceThreshold)
      {
        moveForward();
        if ((stepsMoved-lastSteps >= 70) && (stepsMoved-lastSteps <= 80)){
          flashColor();
        }
      }
      else
      {

        action = false;
        forward = false;
        moveStop();
        if (stepsMoved-lastSteps >= 100){
          updateForward();
        }

      }
      break; 
  }
}

void moveForward()
{ 
  if((calculateLeftDistance() <= 11.0) && (calculateRightDistance() <= 11.0)){
      
      actualDistance = saturationFunctionDistance(calculateFrontDistance());  //Front
      actualDistance2 = saturationFunctionDistance(calculateRightDistance()); //Right
      actualDistance3 = saturationFunctionDistance(calculateLeftDistance());  //Left
      
      error = desiredDistance - actualDistance; //Front
      controlSignal = Pterm*error;
    
      error2 = desiredDistance2 - actualDistance2;  //Right
      controlSignal2 = Pterm2*error2;

      error3 = desiredDistance3 - actualDistance3;  //Left
      controlSignal3 = Pterm3*error3;
      
      outputControlSignalRight = saturationFunctionRight(controlSignal);
      outputControlSignalLeft = saturationFunctionLeft(controlSignal);
    
      leftServo.writeMicroseconds(outputControlSignalLeft-controlSignal2+controlSignal3);
      rightServo.writeMicroseconds(outputControlSignalRight-controlSignal2+controlSignal3);
      
  }
  else if((calculateLeftDistance() <= 11.0) && (calculateRightDistance() > 11.0)){
    
      actualDistance = saturationFunctionDistance(calculateFrontDistance());  //Front
      actualDistance3 = saturationFunctionDistance(calculateLeftDistance());  //Left
      
      error = desiredDistance - actualDistance; //Front
      controlSignal = Pterm*error;

      error3 = desiredDistance3 - actualDistance3;  //Left
      controlSignal3 = Pterm3*error3;
      
      outputControlSignalRight = saturationFunctionRight(controlSignal);
      outputControlSignalLeft = saturationFunctionLeft(controlSignal);
    
      leftServo.writeMicroseconds(outputControlSignalLeft+controlSignal3);
      rightServo.writeMicroseconds(outputControlSignalRight+controlSignal3);
      
  }
  else if((calculateRightDistance() <= 11.0) && (calculateLeftDistance() > 11.0)){
    
      actualDistance = saturationFunctionDistance(calculateFrontDistance());  //Front
      actualDistance2 = saturationFunctionDistance(calculateRightDistance()); //Right
      
      error = desiredDistance - actualDistance; //Front
      controlSignal = Pterm*error;
    
      error2 = desiredDistance2 - actualDistance2;  //Right
      controlSignal2 = Pterm2*error2;

      outputControlSignalRight = saturationFunctionRight(controlSignal);
      outputControlSignalLeft = saturationFunctionLeft(controlSignal);
    
      leftServo.writeMicroseconds(outputControlSignalLeft-controlSignal2);
      rightServo.writeMicroseconds(outputControlSignalRight-controlSignal2);
   
  }
  else{ //Straight Forward without Proportional feedback

    if (calculateFrontDistance() <= desiredDistance){
      moveStop();
    }
    else
    {
      leftServo.writeMicroseconds(1550); 
      rightServo.writeMicroseconds(1443);
    }
  }
}

void moveStop()
{
  leftServo.writeMicroseconds(1500); 
  rightServo.writeMicroseconds(1500);
}

void turnLeft()  //90 degree left turn
{
  leftServo.writeMicroseconds(1450); 
  rightServo.writeMicroseconds(1450);
}

void turnRight()  //90 degree right turn
{
  leftServo.writeMicroseconds(1550); 
  rightServo.writeMicroseconds(1550);
}

void uTurn()  //180 degree left turn
{
  leftServo.writeMicroseconds(1550); 
  rightServo.writeMicroseconds(1550);

}


double calculateRightDistance()
{
  //right sensor
  int sensorValue  = analogRead(ShortRightSensor);
  rightSensorSamples.add(sensorValue);
  long m = rightSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);

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

double calculateLeftDistance()
{
  int sensorValue  = analogRead(ShortLeftSensor);
  leftSensorSamples.add(sensorValue);
  long m = leftSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);
   

  return distance; 

}

grid checkWallFront(){

  double distance = calculateFrontDistance();

  
  if(distance < 9.0){
    switch (robot.theta){
      case NORTH:{
          return (world.square[robot.x - 1][robot.y] == FIXEDWALL) ? FIXEDWALL : WALL;
        break;
      }
      case EAST:{
          return (world.square[robot.x][robot.y + 1] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
      case SOUTH:{
          return (world.square[robot.x + 1][robot.y] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
      case WEST:{
          return (world.square[robot.x][robot.y - 1] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
    }
  }
  else{
    switch (robot.theta){
      case NORTH:{
        if(world.square[robot.x - 1][robot.y] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x - 1][robot.y] == VISITED) ? VISITED : FREE;
        }
        break;
      }
      case EAST:{
        if(world.square[robot.x][robot.y + 1] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x][robot.y + 1] == VISITED) ? VISITED : FREE; 
        }
        break;
      }
      case SOUTH:{
        if(world.square[robot.x + 1][robot.y] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x + 1][robot.y] == VISITED) ? VISITED : FREE; 
        }
        break;
      }
      case WEST:{
        if(world.square[robot.x][robot.y - 1] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x][robot.y - 1] == VISITED) ? VISITED : FREE; 
        }
        break;
      }
    }
  }
}

grid checkWallLeft(){

  double distance = calculateLeftDistance();

  if(distance < 9.0){
    switch (robot.theta){
      case NORTH:{
          return (world.square[robot.x][robot.y - 1] == FIXEDWALL) ? FIXEDWALL : WALL;
        break;
      }
      case EAST:{
          return (world.square[robot.x - 1][robot.y] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
      case SOUTH:{
          return (world.square[robot.x][robot.y + 1] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
      case WEST:{
          return (world.square[robot.x + 1][robot.y] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
    }
  }
  else{
    switch (robot.theta){
      case NORTH:{
        if(world.square[robot.x][robot.y - 1] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x][robot.y - 1] == VISITED) ? VISITED : FREE;
        }
        break;
      }
      case EAST:{
        if(world.square[robot.x - 1][robot.y] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x - 1][robot.y] == VISITED) ? VISITED : FREE;
        }
        break;
      }
      case SOUTH:{
        if(world.square[robot.x][robot.y + 1] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x][robot.y + 1] == VISITED) ? VISITED : FREE;
        }
        break;
      }
      case WEST:{
        if(world.square[robot.x + 1][robot.y] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x + 1][robot.y] == VISITED) ? VISITED : FREE;
        }
        break;
      }
    }
  }
}

grid checkWallRight(){

  double distance = calculateRightDistance();

  if(distance < 9.0){
    switch (robot.theta){
      case NORTH:{
          return (world.square[robot.x][robot.y + 1] == FIXEDWALL) ? FIXEDWALL : WALL;
        break;
      }
      case EAST:{
          return (world.square[robot.x + 1][robot.y] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
      case SOUTH:{
          return (world.square[robot.x][robot.y - 1] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
      case WEST:{
          return (world.square[robot.x - 1][robot.y] == FIXEDWALL) ? FIXEDWALL : WALL; 
        break;
      }
    }
  }
  else{
    switch (robot.theta){
      case NORTH:{
        if(world.square[robot.x][robot.y + 1] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x][robot.y + 1] == VISITED) ? VISITED : FREE;
        }
        break;
      }
      case EAST:{
        if(world.square[robot.x + 1][robot.y] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x + 1][robot.y] == VISITED) ? VISITED : FREE;
        }
        break;
      }
      case SOUTH:{
        if(world.square[robot.x][robot.y - 1] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x][robot.y - 1] == VISITED) ? VISITED : FREE;
        }
        break;
      }
      case WEST:{
        if(world.square[robot.x - 1][robot.y] == FIXEDWALL){
          return FIXEDWALL;
        }
        else{
          return (world.square[robot.x - 1][robot.y] == VISITED) ? VISITED : FREE;
        } 
        break;
      }
    }
  }
}

void initializeWorldMap(){

  //Initializing map with 0s
  int i;
  int j;
  for (i = 0; i < 9; i++){
      for (j = 0; j < 9; j++){
        world.square[i][j] = FREE;
      }
  }

  //Adding boundary walls to world map
  for (i = 0; i < 9; i++){
    world.square[0][i] = FIXEDWALL;
    world.square[8][i] = FIXEDWALL;
    world.square[i][0] = FIXEDWALL;
    world.square[i][8] = FIXEDWALL;
  }

  if(world.maptype == 0){
  //Walls for maze 1
    world.square[2][1] = FIXEDWALL;
    world.square[2][2] = FIXEDWALL;
    world.square[2][3] = FIXEDWALL;
    world.square[2][4] = FIXEDWALL;
    world.square[2][6] = FIXEDWALL;
    world.square[3][4] = FIXEDWALL;
    world.square[3][6] = FIXEDWALL;
    world.square[4][2] = FIXEDWALL;
    world.square[4][4] = FIXEDWALL;
    world.square[4][6] = FIXEDWALL;
    world.square[5][2] = FIXEDWALL;
    world.square[5][6] = FIXEDWALL;
    world.square[6][2] = FIXEDWALL;
    world.square[6][3] = FIXEDWALL;
    world.square[6][4] = FIXEDWALL;
    world.square[6][5] = FIXEDWALL;
    world.square[6][6] = FIXEDWALL;
  }
  else if(world.maptype == 1){
    //Walls for maze 2
    world.square[2][1] = FIXEDWALL;
    world.square[2][2] = FIXEDWALL;
    world.square[2][3] = FIXEDWALL;
    world.square[2][4] = FIXEDWALL;
    world.square[2][6] = FIXEDWALL;
    world.square[3][4] = FIXEDWALL;
    world.square[3][6] = FIXEDWALL;
    world.square[4][2] = FIXEDWALL;
    world.square[4][3] = FIXEDWALL;
    world.square[4][4] = FIXEDWALL;
    world.square[4][6] = FIXEDWALL;
    world.square[5][2] = FIXEDWALL;
    world.square[5][6] = FIXEDWALL;
    world.square[6][2] = FIXEDWALL;
    world.square[6][4] = FIXEDWALL;
    world.square[6][5] = FIXEDWALL;
    world.square[6][6] = FIXEDWALL;
  }
  else if(world.maptype == 2){
    //Walls for maze 3
    world.square[2][2] = FIXEDWALL;
    world.square[2][3] = FIXEDWALL;
    world.square[2][4] = FIXEDWALL;
    world.square[2][5] = FIXEDWALL;
    world.square[2][6] = FIXEDWALL;
    world.square[3][6] = FIXEDWALL;
    world.square[4][2] = FIXEDWALL;
    world.square[4][6] = FIXEDWALL;
    world.square[5][2] = FIXEDWALL;
    world.square[6][2] = FIXEDWALL;
    world.square[6][3] = FIXEDWALL;
    world.square[6][4] = FIXEDWALL;
    world.square[6][5] = FIXEDWALL;
    world.square[6][6] = FIXEDWALL;
  }
}

void initializeRobot(){

  world.targetGridNumber = world.startGrid;

  getTargetCoordinates();
  robot.x = world.targetX;
  robot.y = world.targetY;

  robot.sensorLeft = world.square[robot.x - 1][robot.y];;
  robot.sensorFront = world.square[robot.x][robot.y + 1];
  robot.sensorRight = world.square[robot.x + 1][robot.y];

  robot.theta = world.startHeading;

  updateRobotCompass();

  //Updating world map square grid with VISITED location
  world.square[robot.x][robot.y] = VISITED;
  
}

void updateRobotCompass(){
  
  switch (robot.theta){
    case NORTH:{
      robot.north = world.square[robot.x - 1][robot.y];
      robot.west = world.square[robot.x][robot.y - 1];
      robot.east = world.square[robot.x][robot.y + 1];
      robot.south = NOTKNOWN;
      break;
    }
    case EAST:{
      robot.north = world.square[robot.x - 1][robot.y];
      robot.west = NOTKNOWN;
      robot.east = world.square[robot.x][robot.y + 1];
      robot.south = world.square[robot.x + 1][robot.y];
      break;
    }

    case SOUTH:{
      robot.north = NOTKNOWN;
      robot.west = world.square[robot.x][robot.y - 1];
      robot.east = world.square[robot.x][robot.y + 1];
      robot.south = world.square[robot.x + 1][robot.y];
      break;
    }
    case WEST:{
      robot.north = world.square[robot.x - 1][robot.y];
      robot.west = world.square[robot.x][robot.y - 1];
      robot.east = NOTKNOWN;
      robot.south = world.square[robot.x + 1][robot.y];
      break;
    }
  }
  
}



void updateWorldMap(){

  //Placing robot on world map

  switch (robot.theta){
    case NORTH:{
      world.square[robot.x][robot.y - 1] = robot.sensorLeft;
      world.square[robot.x - 1][robot.y] = robot.sensorFront;
      world.square[robot.x][robot.y + 1] = robot.sensorRight;
      break;
    }
    case EAST:{
      world.square[robot.x - 1][robot.y] = robot.sensorLeft;
      world.square[robot.x][robot.y + 1] = robot.sensorFront;
      world.square[robot.x + 1][robot.y] = robot.sensorRight;
      break;
    }
    case SOUTH:{
      world.square[robot.x][robot.y + 1] = robot.sensorLeft;
      world.square[robot.x + 1][robot.y] = robot.sensorFront;
      world.square[robot.x][robot.y - 1] = robot.sensorRight;
      break;
    }
    case WEST:{
      world.square[robot.x + 1][robot.y] = robot.sensorLeft;
      world.square[robot.x][robot.y - 1] = robot.sensorFront;
      world.square[robot.x - 1][robot.y] = robot.sensorRight;
      break;
    }
  }
}

//Print LCD on Robot
void printLCD(){
  
  Serial.print("=========== LCD OUTPUT ===========\n");
  int i;
  int j;
  for (i = 1; i < 9; i = i + 2){
      for (j = 1; j < 9; j = j + 2){
        if(world.square[i][j] == VISITED){
      Serial.print("X"); 
    }
    else{
      Serial.print("0"); 
    }
    

      }
  }
  int gridNo = updateGridNumber();

  //Printing LCD info on Serial monitor
  Serial.print("\n");
  Serial.print("G");
  Serial.print(gridNo);
  Serial.print("\t");
  Serial.print("W");
  Serial.print(char(robot.west));
  Serial.print("\t");
  Serial.print("N");
  Serial.print(char(robot.north));
  Serial.print("\t");
  Serial.print("E");
  Serial.print(char(robot.east));
  Serial.print("\t");
  Serial.print("S");
  Serial.print(char(robot.south));
  Serial.print("\t");
  Serial.print("\n==================================\n");

  //Printing on LCD
  lcd.clear();
  //Second line
  //Grid No.
  lcd.setCursor(0, 1);
  lcd.print("G");
  lcd.setCursor(1, 1);
  lcd.print(gridNo);

  //West
  lcd.setCursor(3, 1);
  lcd.print("W");
  lcd.setCursor(4, 1);
  if(robot.west == FIXEDWALL){lcd.print("X");}else if(robot.west == VISITED){lcd.print("0");}else if(robot.west == NOTKNOWN){lcd.print("U");}else{lcd.print("0");}

  //North
  lcd.setCursor(6, 1);
  lcd.print("N");
  lcd.setCursor(7, 1);
  if(robot.north == FIXEDWALL){lcd.print("X");}else if(robot.north == VISITED){lcd.print("0");}else if(robot.north == NOTKNOWN){lcd.print("U");}else{lcd.print("0");}

  //South
  lcd.setCursor(9, 1);
  lcd.print("S");
  lcd.setCursor(10, 1);
  if(robot.south == FIXEDWALL){lcd.print("X");}else if(robot.south == VISITED){lcd.print("0");}else if(robot.south == NOTKNOWN){lcd.print("U");}else{lcd.print("0");}

  //East
  lcd.setCursor(12, 1);
  lcd.print("E");
  lcd.setCursor(13, 1);
  if(robot.east == FIXEDWALL){lcd.print("X");}else if(robot.east == VISITED){lcd.print("0");}else if(robot.east == NOTKNOWN){lcd.print("U");}else{lcd.print("0");}
  
  int counter = 0;
  for (i = 1; i <= 7; i = i + 2){
      for (j = 1; j <= 7; j = j + 2){
        if(world.square[i][j] == VISITED){
          lcd.setCursor(counter, 0);
          lcd.print("X");
          counter++; 
        }
        else{
          lcd.setCursor(counter, 0);
          lcd.print("0");
          counter++; 
        }
      }
  }

}

void printWorldMap(){
  
  //Printing Map and position of robot
  Serial.print("Iteration # \n");
  Serial.println(iterationCounter);
  Serial.print("=== World Map ===\n");
  int i;
  int j;
  for (i = 0; i < 9; i++){
      for (j = 0; j < 9; j++){
        Serial.print(char(world.square[i][j]));
        if (j == 8){
          Serial.print("\n");
        }
        else{
          Serial.print(" ");
        }
      }
  }
  Serial.print("=================\n");   
}


int updateGridNumber(){
  if (robot.x == 1 && robot.y == 1){
    return 1;
  }
  if (robot.x == 1 && robot.y == 3){
    return 2;
  }
  if (robot.x == 1 && robot.y == 5){
    return 3;
  }
  if (robot.x == 1 && robot.y == 7){
    return 4;
  }
  if (robot.x == 3 && robot.y == 1){
    return 5;
  }
  if (robot.x == 3 && robot.y == 3){
    return 6;
  }
  if (robot.x == 3 && robot.y == 5){
    return 7;
  }
  if (robot.x == 3 && robot.y == 7){
    return 8;
  }
  if (robot.x == 5 && robot.y == 1){
    return 9;
  }
  if (robot.x == 5 && robot.y == 3){
    return 10;
  }
  if (robot.x == 5 && robot.y == 5){
    return 11;
  }
  if (robot.x == 5 && robot.y == 7){
    return 12;
  }
  if (robot.x == 7 && robot.y == 1){
    return 13;
  }
  if (robot.x == 7 && robot.y == 3){
    return 14;
  }
  if (robot.x == 7 && robot.y == 5){
    return 15;
  }
  if (robot.x == 7 && robot.y == 7){
    return 16;
  }
}

//Check squares in the world map to see if they all have been visited or not
bool allSquaresVisited(){

  int gridCounter = 0;
  int i;
  int j;
  for (i = 1; i < 9; i = i + 2){
      for (j = 1; j < 9; j = j + 2){
        if (world.square[i][j] == VISITED){
      gridCounter++;
    }
      }
  }
  if(gridCounter == 16){
    return true;
  }
  else{
    return false;
  }

}

void checkSensors(){

  robot.sensorLeft = checkWallLeft();
  robot.sensorFront = checkWallFront();
  robot.sensorRight = checkWallRight();

}

void updateForward(){
  
//  //Remove robot footprint from old grid
  
  switch (robot.theta){
    case NORTH:{
      robot.x--;
      world.square[robot.x][robot.y] = VISITED;
      robot.x--;
      world.square[robot.x][robot.y] = VISITED;
      break;
    }
    case EAST:{
      robot.y++;
      world.square[robot.x][robot.y] = VISITED;
      robot.y++;
      world.square[robot.x][robot.y] = VISITED;
      break;
    }

    case SOUTH:{
      robot.x++;
      world.square[robot.x][robot.y] = VISITED;
      robot.x++;
      world.square[robot.x][robot.y] = VISITED;
      break;
    }
    case WEST:{
      robot.y--;
      world.square[robot.x][robot.y] = VISITED;
      robot.y--;
      world.square[robot.x][robot.y] = VISITED;
      break;
    }
  }

}

void updateLeft(){
  switch (robot.theta){
    case NORTH:{
      robot.theta = WEST;
      break;
    }
    case EAST:{
      robot.theta = NORTH;
      break;
    }

    case SOUTH:{
      robot.theta = EAST;
      break;
    }
    case WEST:{
      robot.theta = SOUTH;
      break;
    }
  }
}

void updateRight(){

  switch (robot.theta){
    case NORTH:{
      robot.theta = EAST;
      break;
    }
    case EAST:{
      robot.theta = SOUTH;
      break;
    }

    case SOUTH:{
      robot.theta = WEST;
      break;
    }
    case WEST:{
      robot.theta = NORTH;
      break;
    }
  }
}

void updateUturn(){

  switch (robot.theta){
    case NORTH:{
      robot.theta = SOUTH;
      break;
    }
    case EAST:{
      robot.theta = WEST;
      break;
    }

    case SOUTH:{
      robot.theta = NORTH;
      break;
    }
    case WEST:{
      robot.theta = EAST;
      break;
    }
  }
}


void flashColor(){

  colorLast = timeElapsed;
  
  switch (robot.theta){
    case NORTH:{
      lcd.setBacklight(BLUE);
      break;
    }
    case EAST:{
      lcd.setBacklight(RED);
      break;
    }

    case SOUTH:{
      lcd.setBacklight(YELLOW);
      break;
    }
    case WEST:{
      lcd.setBacklight(GREEN);
      break;
    }
  }
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

int encoderCount(){

  int averageCount = 0;
  int lEncVal = digitalRead(lEncPin);
  int rEncVal = digitalRead(rEncPin);
  
  if (lEncVal != lEncLast) lEncCount++;
  if (rEncVal != rEncLast) rEncCount++;

  lEncLast = lEncVal;
  rEncLast = rEncVal;
  
  averageCount = (lEncCount + rEncCount)/2;
  return averageCount;
}

void getTargetCoordinates(){
  switch(world.targetGridNumber){
    case 1:
      world.targetX = 1;
      world.targetY = 1;
      break;
    case 2:
      world.targetX = 1;
      world.targetY = 3;
      break;
    case 3:
      world.targetX = 1;
      world.targetY = 5;
      break; 
    case 4:
      world.targetX = 1;
      world.targetY = 7;
      break;
    case 5:
      world.targetX = 3;
      world.targetY = 1;
      break;
    case 6:
      world.targetX = 3;
      world.targetY = 3;
      break;
    case 7:
      world.targetX = 3;
      world.targetY = 5;
      break;
    case 8:
      world.targetX = 3;
      world.targetY = 7;
      break;
    case 9:
      world.targetX = 5;
      world.targetY = 1;
      break;
    case 10:
      world.targetX = 5;
      world.targetY = 3;
      break;
    case 11:
      world.targetX = 5;
      world.targetY = 5;
      break;
    case 12:
      world.targetX = 5;
      world.targetY = 7;
      break;
    case 13:
      world.targetX = 7;
      world.targetY = 1;
      break;
    case 14:
      world.targetX = 7;
      world.targetY = 3;
      break;
    case 15:
      world.targetX = 7;
      world.targetY = 5;
      break;
    case 16:
      world.targetX = 7;
      world.targetY = 7;
      break;
  }
}

void calculateTargetHeading(){
  if(world.targetX != robot.x || world.targetY != robot.y){
    if(world.targetX > robot.x){
      world.targetHeading = SOUTH;
    }
    if(world.targetX < robot.x){
      world.targetHeading = NORTH;
    }
    if(world.targetY > robot.y){
      world.targetHeading = EAST;
    }
    if(world.targetY < robot.y){
      world.targetHeading = WEST;
    }
  }
}

void switchPathFollowerState(){
  
    lastSteps = encoderCount();
    
    if (forward == true){
      state = 4;  //Forward move
      last = timeElapsed;
      action = true;
    }
    else if (world.targetHeading == robot.theta){
      state = 4;  //Forward move
      last = timeElapsed;
      action = true;
      forward = true;
    }
    else{
      switch(robot.theta){
        case NORTH:
          switch(world.targetHeading){
            case WEST:
              //Turn right
              state = 2;  //Right 90 degree turn
              last = timeElapsed;
              action = true;
              right = true;
              break;
            case SOUTH:
              //U turn
              state = 3;  //90 degree uturn
              last = timeElapsed;
              action = true;
              uturn = true;
              break;
            case EAST:
              //Turn left
              state = 1;  //Left 90 degree turn
              last = timeElapsed;
              action = true;
              left = true;
              break;
          }
          break;
        case WEST:
          switch(world.targetHeading){
            case NORTH:
              //Turn right
              state = 2;  //Right 90 degree turn
              last = timeElapsed;
              action = true;
              right = true;
              break;
            case SOUTH:
              //Turn left
              state = 1;  //Left 90 degree turn
              last = timeElapsed;
              action = true;
              left = true;
              break;
            case EAST:
              //U turn
              state = 3;  //90 degree uturn
              last = timeElapsed;
              action = true;
              uturn = true;
              break;
          }
          break;
        case SOUTH:
          switch(world.targetHeading){
            case WEST:
              //Turn right
              state = 2;  //Right 90 degree turn
              last = timeElapsed;
              action = true;
              right = true; 
              break;
            case NORTH:
              //U turn
              state = 3;  //90 degree uturn
              last = timeElapsed;
              action = true;
              uturn = true;
              break;
            case EAST:
              //Turn left
              state = 1;  //Left 90 degree turn
              last = timeElapsed;
              action = true;
              left = true;
              break;
          }
          break;
        case EAST:
          switch(world.targetHeading){
            case WEST:
              //U turn
              state = 3;  //90 degree uturn
              last = timeElapsed;
              action = true;
              uturn = true;
              break;
            case SOUTH:
              //Turn Right
              state = 2;  //Right 90 degree turn
              last = timeElapsed;
              action = true;
              right = true;
              break;
            case NORTH:
              //Turn left
              state = 1;  //Left 90 degree turn
              last = timeElapsed;
              action = true;
              left = true;
              break;
          }
          break;
      }

    }
}

void cleanWorldMap(){
  
  for (int i = 0; i < 9; i++){
      for (int j = 0; j < 9; j++){
        if ((world.square[i][j] != WALL) && (world.square[i][j] != FIXEDWALL)){
          world.square[i][j] = VISITED;
        }
      }
  }
}

void runFloodFill(){
  world.square[world.targetX][world.targetY] = 0;  //Mark goal grid 0. FIX THIS!
  do{
    //Left check
    if((world.square[world.targetX][world.targetY - 1] != WALL) && (world.square[world.targetX][world.targetY - 1] != FIXEDWALL) && (world.square[world.targetX][world.targetY - 2] == VISITED)){
      world.square[world.targetX][world.targetY - 2] = (world.square[world.targetX][world.targetY] + 1);  //Mark grid as x + 1
      floodFillAdjacenctGrids.push(returnGridNumber((world.targetX), (world.targetY - 2))); //Push in queue

    }
    //Front check
    if((world.square[world.targetX - 1][world.targetY] != WALL) && (world.square[world.targetX - 1][world.targetY] != FIXEDWALL) && (world.square[world.targetX - 2][world.targetY] == VISITED)){
      world.square[world.targetX - 2][world.targetY] = (world.square[world.targetX][world.targetY] + 1);  //Mark grid as x + 1
      floodFillAdjacenctGrids.push(returnGridNumber((world.targetX - 2), (world.targetY))); //Push in queue
    }
    //Right check
    if((world.square[world.targetX][world.targetY + 1] != WALL) && (world.square[world.targetX][world.targetY + 1] != FIXEDWALL) && (world.square[world.targetX][world.targetY + 2] == VISITED)){
      world.square[world.targetX][world.targetY + 2] = (world.square[world.targetX][world.targetY] + 1);  //Mark grid as x + 1
      floodFillAdjacenctGrids.push(returnGridNumber((world.targetX), (world.targetY + 2))); //Push in queue
    }
    //Back check
    if((world.square[world.targetX + 1][world.targetY] != WALL) && (world.square[world.targetX + 1][world.targetY] != FIXEDWALL) && (world.square[world.targetX + 2][world.targetY] == VISITED)){
      world.square[world.targetX + 2][world.targetY] = (world.square[world.targetX][world.targetY] + 1);  //Mark grid as x + 1
      floodFillAdjacenctGrids.push(returnGridNumber((world.targetX + 2), (world.targetY))); //Push in queue
    }
    world.targetGridNumber = floodFillAdjacenctGrids.pop();
    getTargetCoordinates();
    
  }while(world.targetGridNumber != world.startGrid);
}

int returnGridNumber(int x, int y){
  if (x == 1 && y == 1){
    return 1;
  }
  if (x == 1 && y == 3){
    return 2;
  }
  if (x == 1 && y == 5){
    return 3;
  }
  if (x == 1 && y == 7){
    return 4;
  }
  if (x == 3 && y == 1){
    return 5;
  }
  if (x == 3 && y == 3){
    return 6;
  }
  if (x == 3 && y == 5){
    return 7;
  }
  if (x == 3 && y == 7){
    return 8;
  }
  if (x == 5 && y == 1){
    return 9;
  }
  if (x == 5 && y == 3){
    return 10;
  }
  if (x == 5 && y == 5){
    return 11;
  }
  if (x == 5 && y == 7){
    return 12;
  }
  if (x == 7 && y == 1){
    return 13;
  }
  if (x == 7 && y == 3){
    return 14;
  }
  if (x == 7 && y == 5){
    return 15;
  }
  if (x == 7 && y == 7){
    return 16;
  }
}

void floodFill(){
    cleanWorldMap();
    world.targetGridNumber = world.goalGrid;
    getTargetCoordinates();
    runFloodFill();
}

void runShortestPath(){
  int smallest;
  shortestPath.push(world.startGrid);
  world.targetGridNumber = world.startGrid;
  getTargetCoordinates();
  smallest = world.square[world.targetX][world.targetY];
  
  while(smallest != 0)
  {
    //Left check
    if((world.square[world.targetX][world.targetY - 1] != WALL) && (world.square[world.targetX][world.targetY - 1] != FIXEDWALL) && (world.square[world.targetX][world.targetY - 2] < world.square[world.targetX][world.targetY])){
      smallest = world.square[world.targetX][world.targetY - 2];  //Mark left adjacent grid value as smallest
      world.targetGridNumber = returnGridNumber((world.targetX), (world.targetY - 2));
      shortestPath.push(world.targetGridNumber); //Push in queue
      getTargetCoordinates();
    }
    //Front check
    else if((world.square[world.targetX - 1][world.targetY] != WALL) && (world.square[world.targetX - 1][world.targetY] != FIXEDWALL) && (world.square[world.targetX - 2][world.targetY] < world.square[world.targetX][world.targetY])){
      smallest = world.square[world.targetX - 2][world.targetY];  //Mark front adjacent grid value as smallest
      world.targetGridNumber = returnGridNumber((world.targetX - 2), (world.targetY));
      shortestPath.push(world.targetGridNumber); //Push in queue
      getTargetCoordinates();
    }
    //Right check
    else if((world.square[world.targetX][world.targetY + 1] != WALL) && (world.square[world.targetX][world.targetY + 1] != FIXEDWALL) && (world.square[world.targetX][world.targetY + 2] < world.square[world.targetX][world.targetY])){
      smallest = world.square[world.targetX][world.targetY + 2];  //Mark front adjacent grid value as smallest
      world.targetGridNumber = returnGridNumber((world.targetX), (world.targetY + 2));
      shortestPath.push(world.targetGridNumber); //Push in queue
      getTargetCoordinates();
    }
    //Back check
    else if((world.square[world.targetX + 1][world.targetY] != WALL) && (world.square[world.targetX + 1][world.targetY] != FIXEDWALL) && (world.square[world.targetX + 2][world.targetY] < world.square[world.targetX][world.targetY])){
      smallest = world.square[world.targetX + 2][world.targetY];  //Mark front adjacent grid value as smallest
      world.targetGridNumber = returnGridNumber((world.targetX + 2), (world.targetY));
      shortestPath.push(world.targetGridNumber); //Push in queue
      getTargetCoordinates();
    }  
  }
}
