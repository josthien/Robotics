#include <Adafruit_RGBLCDShield.h>

// RGB SHIELD
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

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


void setup() {
  
  //SET RGB SHIELD
  Serial.begin(9600);
  lcd.begin(16, 2);
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

void loop() {
  
    // Setting red filtered photodiodes to be read
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    fRed = pulseIn(sensorOut, LOW);
    delay(100);
  
    // Setting Green filtered photodiodes to be read
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    fGreen = pulseIn(sensorOut, LOW);
    delay(100);
  
    // Setting Blue filtered photodiodes to be read
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    fBlue = pulseIn(sensorOut, LOW);
    delay(100);

    if (isRed())
    {
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("RED");
      lcd.setBacklight(RED);
    }

    if (isBlue())
    {
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("BLUE");
      lcd.setBacklight(BLUE);
    }

    if (isBrown())
    {
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("BROWN");
      lcd.setBacklight(YELLOW);
    }

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
