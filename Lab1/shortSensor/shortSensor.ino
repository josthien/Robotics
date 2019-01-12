/* Description
  The robot is equipped with 3 IR short sensors located in front, left and right side of the robot
  The short sensors' range is 2-10 inches. The code below will convert values measured by the sensors to 
  actual distance in inches and show the results on the LCD
*/
#include <RunningMedian.h>
#include <Adafruit_RGBLCDShield.h>

const int ShortFrontSensor = A0;
const int ShortLeftSensor = A1; 
const int ShortRightSensor = A2;

RunningMedian frontSensorSamples = RunningMedian(10);
RunningMedian leftSensorSamples = RunningMedian(10);
RunningMedian rightSensorSamples = RunningMedian(10);

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

//set up the lcd of the robot
void setup() 
{
  lcd.begin(16, 2); 
}

//this function acts as main function for the program
//the robot will continuously performs the functions called in loop function
void loop() 
{
  calculateFrontDistance();
  calculateLeftDistance();
  calculateRightDistance();
  delay(200);
  lcd.clear();
}

// calculate the distance from robot to obstacle located in front of the robot by converting the sensor value to distance
void calculateFrontDistance()
{
  int sensorValue  = analogRead(ShortFrontSensor);
  frontSensorSamples.add(sensorValue);
  long m = frontSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0); //calculate the sensor value
  double distance = 5.0146*pow(voltage, -1.055); //convert to actual distance
   
  lcd.setCursor(0,0);
  lcd.print("F:");
  lcd.setCursor(2,0);
  if (distance >= 2.0 && distance <= 10.0)
    lcd.print(distance);
  else
    lcd.print("XXXX");
}

// calculate the distance from robot to obstacle located in the left-hand side of the robot 
// by converting the sensor value to distance
void calculateLeftDistance()
{
  int sensorValue  = analogRead(ShortLeftSensor);
  leftSensorSamples.add(sensorValue);
  long m = leftSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0); //calculate the sensor value
  double distance = 5.0146*pow(voltage, -1.055); //convert to actual distance
  
  lcd.setCursor(9,0);
  lcd.print("L:");
  lcd.setCursor(11,0);
  if (distance >= 2.0 && distance <= 10.0)
    lcd.print(distance);
  else
    lcd.print("XXXX");
}

// calculate the distance from robot to obstacle located in the right-hand side of the robot 
// by converting the sensor value to distance
void calculateRightDistance()
{
  //right sensor
  int sensorValue  = analogRead(ShortRightSensor);
  rightSensorSamples.add(sensorValue);
  long m = rightSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0); //calculate the sensor value
  double distance = 5.0146*pow(voltage, -1.055); //convert to actual distance
  
  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.setCursor(2,1);
  if (distance >= 2.0 && distance <= 10.0)
    lcd.print(distance);
  else
    lcd.print("XXXX");
}

// printing out median, highest, lowest and count values on the serial monitor
void printSerialMonitor()
{
  // For printing out median, highest, lowest and count values on the serial monitor 
  float h = frontSensorSamples.getHighest();
  float l = frontSensorSamples. getLowest();
  int c = frontSensorSamples.getCount();

  Serial.println(m);
  Serial.println(h);
  Serial.println(l);
  Serial.println(c);
}
