#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
int device = 1;
HMC5883L_Simple Compass;
int rtcaddress = 0x68;
int compassaddress = 0x3D;
tmElements_t tm;
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Compass.SetDeclination(0, 42, 'E');  
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_Y_NORTH);

  Serial.println("DS1307RTC Read Test");
  Serial.println("-------------------");
  Wire.beginTransmission(rtcaddress);
  Wire.beginTransmission(compassaddress);
  Wire.endTransmission();
  Wire.endTransmission();
}


void loop()
{
 Wire.beginTransmission(compassaddress);
 Wire.endTransmission();
 float heading = Compass.GetHeadingDegrees();
   
 Serial.print("Heading: \t");
 Serial.println( heading );   
 delay(1000);
   
 
 Wire.beginTransmission(rtcaddress);
 Wire.endTransmission();
  if (RTC.read(tm)) {
    Serial.print("Ok, Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
  } 
  else {
    if (RTC.chipPresent()) {
      bool parse=false;
      bool config=false;
      if (getDate(__DATE__) && getTime(__TIME__)) {
      parse = true;
      
      if (RTC.write(tm)) {
        config = true;
      }
    }

      Serial.begin(9600);
      while (!Serial) ;
      delay(200);
      if (parse && config) {
        Serial.print("DS1307 configured Time=");
        Serial.print(__TIME__);
        Serial.print(", Date=");
        Serial.println(__DATE__);
      } 
      else if (parse) {
        Serial.println("DS1307 Communication Error :-{");
        Serial.println("Please check your circuitry");
      } 
      else {
        Serial.print("Could not parse info from the compiler, Time=\"");
        Serial.print(__TIME__);
        Serial.print("\", Date=\"");
        Serial.print(__DATE__);
        Serial.println("\"");
      }
    } 
    else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
    delay(9000);
  }
  delay(1000);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}
