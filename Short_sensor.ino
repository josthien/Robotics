#include <RunningMedian.h>
#include <Adafruit_RGBLCDShield.h>

const int ShortFrontSensor = A0;
const int ShortLeftSensor = A1; 
const int ShortRightSensor = A2;

RunningMedian frontSensorSamples = RunningMedian(10);
RunningMedian leftSensorSamples = RunningMedian(10);
RunningMedian rightSensorSamples = RunningMedian(10);

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup() {
  lcd.begin(16, 2); 
}

void loop() {
  calculateFrontDistance();
  calculateLeftDistance();
  calculateRightDistance();
  delay(200);
  lcd.clear();

}

void calculateFrontDistance()
{
  int sensorValue  = analogRead(ShortFrontSensor);
  frontSensorSamples.add(sensorValue);
  long m = frontSensorSamples.getMedian();

//  For printing out median, highest, lowest and count values on the serial monitor 
//  float h = frontSensorSamples.getHighest();
//  float l = frontSensorSamples. getLowest();
//  int c = frontSensorSamples.getCount();
//
//  Serial.println(m);
//  Serial.println(h);
//  Serial.println(l);
//  Serial.println(c);
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);
   
  lcd.setCursor(0,0);
  lcd.print("F:");
  lcd.setCursor(2,0);
  if (distance >= 2.0 && distance <= 10.0)
    lcd.print(distance);
  else
    lcd.print("XXXX");
}

void calculateLeftDistance()
{
  int sensorValue  = analogRead(ShortLeftSensor);
  leftSensorSamples.add(sensorValue);
  long m = leftSensorSamples.getMedian();
  
  double voltage = m*(5.0/1023.0);
  double distance = 5.0146*pow(voltage, -1.055);
  
  lcd.setCursor(9,0);
  lcd.print("L:");
  lcd.setCursor(11,0);
  if (distance >= 2.0 && distance <= 10.0)
    lcd.print(distance);
  else
    lcd.print("XXXX");
}

void calculateRightDistance()
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
}
