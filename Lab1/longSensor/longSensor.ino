/* Description
  The robot is equipped with a long sensor located in front of the robot
  The sensor's range is 8-59 inches. The code below will convert values measured by the sensor to 
  actual distance in inches and show the results on the LCD
*/
#include <Adafruit_RGBLCDShield.h>
#include <RunningMedian.h>
#include <Servo.h>  // Include servo library

Servo LServo;  // Declare Left servo
Servo RServo;  // Declare right servo

RunningMedian longSensorSamples = RunningMedian(20);
RunningMedian shortSensorSamples = RunningMedian(10);

const int LongFrontSensor = A3;
const int ShortFrontSensor = A0;
double shortDistance = 0.0;
double longDistance = 0.0;
double averageDistance = 0.0;
double finalDistance = 0.0;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup() {
  lcd.begin(16, 2); 
}

void loop() {
  delay(200);
  lcd.clear();

  longDistance = calculateLongDistance();
  shortDistance = calculateShortDistance();

  if (shortDistance <= 8.00)
  {
    lcd.setCursor(0,0);
    lcd.print("Front:");
    lcd.setCursor(6,0);
    if (shortDistance >= 2.0)
      lcd.print(shortDistance);
    else
      lcd.print("XXXX");
  }
  else if (longDistance >= 10.00 & shortDistance >= 10.00)
  {
    lcd.setCursor(0,0);
    lcd.print("Front:");
    lcd.setCursor(6,0);
    if (longDistance <= 59.0)
      lcd.print(longDistance);
    else
      lcd.print("XXXX");  
  }
  else
  {
    averageDistance = (longDistance + (shortDistance - 0.5))/2;
    lcd.setCursor(0,0);
    lcd.print("Front:");
    lcd.setCursor(6,0);
    lcd.print(averageDistance); 

  }
}

// calculate distance from robot to obstables based on long sensor value
double calculateLongDistance()
{
  double distance;
  int sensorValue  = analogRead(LongFrontSensor);
  longSensorSamples.add(sensorValue);
  long m = longSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);

  if (voltage > 1.50 && voltage < 2.80)
  {
    distance = -6.8945*voltage + 25.647;
  }
  else if (voltage <= 1.50 && voltage >= 0.85)
  {
    distance = 23.707*pow(voltage, -0.998);
  }
  else
  {
    distance = 23.112*pow(voltage, -1.029);
  }
  return distance;
}

// calculate distance from robot to obstables based on short sensor value
double calculateShortDistance()
{
  int sensorValue  = analogRead(ShortFrontSensor);
  shortSensorSamples.add(sensorValue);
  long m = shortSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);

  return distance;
}

