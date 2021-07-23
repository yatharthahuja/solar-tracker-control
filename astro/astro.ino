//Astronomical method pertained sensors input ports:
//RTC DS3231: SDA, SCA Pins same as Magnetometer due to I2C bus arrangement
#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
int device = 1;

//Magnetometer HMC5883L:
const int sda = 20 ;
const int scl = 21 ;

//const int device = 1 ;
const int rtcaddress = 0x68 ;
const int compassaddress = 0x3D ;
//const char *months[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" } ;
const int latitude = 45 ; // Enter latitude of the location in degrees, roughly
const int days_in_month_normal[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 } ;
//const int days_in_month_leap[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 } ;
HMC5883L_Simple Compass ;
tmElements_t tm ;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  //For RTC+Magnetometer
  Serial.print("->Calibrating modules...") ;
  Wire.begin() ;
  Compass.SetDeclination(0, latitude, 'E') ;
  Compass.SetSamplingMode(COMPASS_SINGLE) ;
  Compass.SetScale(COMPASS_SCALE_130) ;
  Compass.SetOrientation(COMPASS_HORIZONTAL_Y_NORTH) ;
  Serial.println("Done!") ;
  Serial.println("DS1307RTC Read Test") ;
  Serial.println("-------------------") ;
  Wire.beginTransmission(rtcaddress) ;
  Wire.beginTransmission(compassaddress) ;
  Wire.endTransmission() ;
  Wire.endTransmission() ;
  Serial.println("Done!") ;
}

//Function to return magnetometer parameter readings as per demand:
int magnetometer ( char* parameter ) {
  
  Wire.beginTransmission(compassaddress) ; //Tell the HMC5883L where to begin reading data
  //Wire.send(0x03) ; //select register 3, X MSB register
  Wire.endTransmission() ;
  
  int yaw, pitch, roll ;
  int heading = Compass.GetHeadingDegrees() ;
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(compassaddress, 6) ;
  if(6<=Wire.available()){
    pitch = Wire.read()<<8 ; //X msb
    pitch |= Wire.read() ; //X lsb
    yaw = Wire.read()<<8 ; //Y msb
    yaw |= Wire.read() ; //Y lsb
    roll = Wire.read()<<8 ; //Z msb
    roll |= Wire.read() ; //Z lsb
  }
  
  if ( parameter == "yaw" ) { // About vertical y-axis
    return yaw ;
  }
  if ( parameter == "pitch" ) { // About horizontal x-axis
    return pitch ;
  }
  else if ( parameter == "roll" ) { // About horizontal z-axis
    return roll ;
  }
  
}

//Function to return RTC day of the year:
int* rtc() {
  
  int day_of_year = 0 ;
  int hour_of_day = 0 ;
  
  Wire.beginTransmission(rtcaddress);
  Wire.endTransmission();
  if (RTC.read(tm)) {
    //int Hour = tm.Hour ;
    //int Minute = tm.Minute ;
    //int Secodn = tm.Second ;
    int Day = tm.Day ;
    int Month = tm.Month ;
    //Serial.print(tmYearToCalendar(tm.Year));
    day_of_year += Day ;
    int i = 0 ;
     while ( i < 11 ) {
      if ( i + 1 == Month ) {
        break ;
      }
      day_of_year += days_in_month_normal[i] ;
      i++ ;
    }
  }
 /* else {
    if (RTC.chipPresent()) {
      bool parse = false;
      bool config = false;
      if (getDate(__DATE__) && getTime(__TIME__)) {
        parse = true;
        if (RTC.write(tm)) {
          config = true;
        }
      }
    } */
  int rtc_data[2] = { day_of_year, hour_of_day } ;
  return rtc_data ;
}

//Function to translate theta to be excuted into voltage commands for the actuator:
int* theta_to_voltage( int theta[] ) {
  int actuation_voltages[2] ;
  
  return actuation_voltages ;
}

//Function to provide theta by breaking down into respective components for each actuator(pitch and roll):
int* componented_theta( int theta[] ) {
  return theta ;
}

//Function to generate astronomical commands
int* astro_commands() {

  //Day of the year(out of 365) - from RTC
  int day_of_year = rtc()[0] ;
  //Hour of the day - from RTC
  int hour_of_day = rtc()[1] ;
  
  //float zenith_angle = 0 ;
  //float incidence_angle = 0 ;//Angle to be achieved in N-S direction. Angle between surface normal and solar radiation
  float clock_angle = ( hour_of_day-12 )*15 ;//Angle to be achieved in E-W direction

  //float azimuthal_angle = 0 ;
  float declination_angle = 23.5 * sin( 360*(284+day_of_year)/365 ) ;
    
  //Heading(yaw) in degrees with respect to true North - from Magnetometer
  int heading = magnetometer( "yaw" ) ;

  //Function to provide theta by breaking down into respective components for each actuator:
  int theta[2] ;//thetas to be executed respectively: theta[0]->Actuator1, theta[1]->Actuator2
  
  //Formulae for the 2 angles(theta below) here  
  theta[0] = acos( ( cos(latitude)*cos(clock_angle)*cos(declination_angle) ) + ( sin(latitude)*sin(declination_angle) ) ) ;//In N-S Direction With respect to vertical
  theta[1] = clock_angle ;//in E-W direction with respect to the vertical 
  
  //theta = componented_theta( theta ) ; //***********TO BE COMPLETED
  //Actuation commands obtained:
  //return theta_to_voltage( theta ) ;
  return theta;
}

void loop() {
  // put your main code here, to run repeatedly:
  int x = astro_commands()[0];
  int y = astro_commands()[1];
  Serial.println(x);
  Serial.println(y);
}
