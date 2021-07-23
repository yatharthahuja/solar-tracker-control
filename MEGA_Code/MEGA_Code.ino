//This code deals with the following:
// Inputs from: IMU, RTC, LDRs
// Execute output on Panel Motors using control algorithms: PID, PIID, PIDD, Fuzzy-ANN, Flat
// **Important for the user to check whether all the variables are declared as per required configuration
//
//Synopsis:
// -> Each arduino::void_loop() encompasses a loop of 3min control+ 15min break to save energy
// -> The void_loop::control_loop() executes selected control algorithm
// -> Built for Arduino MEGA board
//
//FOR REFERENCE:
//Circuit: load_side_ckt proteus file
//Actuator index:
//  Actuator 1 -> 8-inch actuator
//  Actuator 2 -> 6-inch actuator

//***C0DE***///

//Libraries
#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Math.h>

//Motor Driver: Actuator2-6inch
const int LEN_2 = 2;
const int REN_2 = 4;
const int LPWM_2 = 3;
const int RPWM_2 = 5;

//Motor Driver: Actuator1-8inch
const int LEN_1 = 9;
const int REN_1 = 8;
const int LPWM_1 = 6;
const int RPWM_1 = 7;

//LDR Array input ports:
const int ref_ldr_in = A0 ;
const int ldr_1_in = A1 ;
const int ldr_2_in = A2 ;
const int ldr_3_in = A3 ;
const int ldr_4_in = A4 ;

//Astronomical method pertained sensors input ports:
//RTC DS3231: SDA, SCA Pins same as Magnetometer due to I2C bus arrangement
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

//Assumed supply Voltage
const int Vbattery = 10 ;

//whether to involve astronomical method in the control:
const bool astronomical_method_reqd = false ;

//whether to involve heading correction method in the control:
const bool heading_interrupt_reqd = false ;

//Control method
const int control_method = 1 ; //Select Option: < 1 for "PID" >, < 2 for "PIID" >, < 3 for "Flat" >, < 4 for "Fuzzy-ANN" >

//Time master loop distribution settings ( in microseconds )
const int sleep_time = 900000 ;//15mins //only sudden change in heading(yaw) causes tracking during this
const int panel_adjustment_time = 180000 ;//3min //timely tracking
const int iteration_time = 2000 ;//1sec //for execution of one control loop

//Setting the desired value of output panel angles' control(theta)
//const int setpoint_theta[2] = { 0, 0 } ;

//Actuator1-8inch: Characteristic parameters:
const float K_1 = 400 ; // N/A
const float R_1 = 348.465 ; // Ohm
const float L_1 = 0.0016931 ; // Henry
const float J_1 = 1829.4 ; // Kg m^2
const float f_1 = 0.00 ; //Negligible

//Actuator2-6inch: Characteristic parameters:
const float K_2 = 600 ; // N/A
const float R_2 = 425.016 ; // Ohm
const float L_2 = 0.000618153 ; // Henry
const float J_2 = 5083.99 ; // Kg m^2
const float f_2 = 0.00 ; //Negligible

//PID constans(for each actuator):
//Actuator-1:
const float Kp_1 = 476.3393 ;
const float Kd_1 = -5.0586 ;
const float Ki_1 = 81.4498 ;
//Actuator-2:
const float Kp_2 = 498.7170 ;
const float Kd_2 = -15.1868 ;
const float Ki_2 = 23.0849 ;


//double Setpoint, Input, Output;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  // put your setup code here, to run once:

  //Initialise serial communication to monitor
  Serial.begin(9600) ;

  Serial.println("INTITATING TRACKING SYSTEM! WELCOME!") ;
  Serial.println("****************************************") ;

  Serial.print("Setting up varibales.....") ;
  Serial.println("Done!") ;

  Serial.print("INITIALISING SYSTEM") ;

  //Inititalising pins
  Serial.print("->Setting up pins...") ;
  pinMode( ref_ldr_in, INPUT) ;
  pinMode( ldr_1_in, INPUT) ;
  pinMode( ldr_2_in, INPUT) ;
  pinMode( ldr_3_in, INPUT) ;
  pinMode( ldr_4_in, INPUT) ;

  //For Actuator1:
  pinMode( LEN_1, OUTPUT ) ;
  pinMode( REN_1, OUTPUT ) ;
  pinMode( LPWM_1, OUTPUT ) ;
  pinMode( RPWM_1, OUTPUT ) ;

  //For Actuator2:
  pinMode( LEN_2, OUTPUT ) ;
  pinMode( REN_2, OUTPUT ) ;
  pinMode( LPWM_2, OUTPUT ) ;
  pinMode( RPWM_2, OUTPUT ) ;
  Serial.println("Done!") ;

  //sensor ports
  pinMode( SDA, INPUT ) ;
  pinMode( SCL, INPUT ) ;

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

void execute_actuator( int command[] ) { //PWM speed to be adjusted as per voltage available in the battery and iterataion time
  //Index-> 0 for Act1-6inch; 1 fror Act2-8inch
  digitalWrite(REN_1,HIGH);
  digitalWrite(LEN_1,HIGH);
  digitalWrite(REN_2,HIGH);
  digitalWrite(LEN_2,HIGH);
 
  //command[0] = ( command[0]/*Vbattery*/ )*255 ;
  //command[1] = ( command[1]/*Vbattery*/ )*255 ;
 
  if ( command[0]>255 ){
    command[0] = 255 ;
  }
  if ( command[0]<-255 ){
    command[0] = -255 ;
  }
  if ( command[1]>255 ){
    command[1] = 255 ;
  }
  if ( command[1]<-255 ){
    command[1] = -255 ;
  }
  
  if ( command[0]>0 ) { //Actuator1
    digitalWrite(REN_1,HIGH);
    digitalWrite(LEN_1,HIGH);
    analogWrite(RPWM_1,255);
    analogWrite(LPWM_1,0);
    Serial.print("Command executed: (Act1) ");
    Serial.println(command[0]);
    delay(1000);
  }
  if ( command[0]<0 ) { //Actuator1
    digitalWrite(REN_1,HIGH);
    digitalWrite(LEN_1,HIGH);
    analogWrite(RPWM_1,0);
    analogWrite(LPWM_1,255);
    Serial.print("Command executed: (Act1) ");
    Serial.println(command[0]);
    delay(1000);
  }

  if ( command[1]>0 ) { //Actuator2
    digitalWrite(REN_2,HIGH);
    digitalWrite(LEN_2,HIGH);
    analogWrite(RPWM_2,255);
    analogWrite(LPWM_2,0);
    Serial.print("Command executed: (Act2) ");
    Serial.println(command[1]);
    delay(1000);
  }
  if ( command[1]<0 ) { //Actuator2
    digitalWrite(REN_2,HIGH);
    digitalWrite(LEN_2,HIGH);
    analogWrite(RPWM_2,0);
    analogWrite(LPWM_2,255);
    Serial.print("Command executed: (Act2) ");
    Serial.println(command[1]);
    delay(1000);
  }
  //Serial.print( "Executed Commands: " ) ; Serial.print( command[0] ) ; Serial.print( "  |  " ) ; Serial.println( command[1] ) ;
  delay(2000) ; //mandatory delay for adjustment and iteration time

  digitalWrite(REN_1,LOW);
  digitalWrite(LEN_1,LOW);
  digitalWrite(REN_2,LOW);
  digitalWrite(LEN_2,LOW);
}

int pid_controller ( int error, int Kp, int Ki, int Kd ) {
  float cmd = error * (Kp + Kd / iteration_time + Ki * iteration_time/*+integral_prior*/) ;
  //Serial.print("Outputs: ") ;
  //Serial.print(Output) ;
  return cmd ;
}

void pidd_controller( int error, int Kp, int Ki, int Kd1, int Kd2 ){
  
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
  theta[0] = (acos( ( cos(latitude)*cos(clock_angle)*cos(declination_angle) ) + ( sin(latitude)*sin(declination_angle) ) ) - magnetometer( "roll" ))/magnetometer( "roll" ) ;//In N-S Direction With respect to vertical
  theta[1] = (clock_angle-magnetometer( "pitch" ))/magnetometer( "pitch" ) ;//in E-W direction with respect to the vertical 

  //theta = componented_theta( theta ) ; //***********TO BE COMPLETED
  //Actuation commands obtained:
  //return theta_to_voltage( theta ) ;
  return theta;
}

//Function to provide normalised LDR errors:
int* LDR_errors() {

  int error[3] = { 0, 0, 0 } ; //Index assignment: 0->ldrref, 1->(ldr1-ldr3), 2->(ldr2-ldr4)
  float ldrref = (analogRead(ref_ldr_in)) ;
  float ldr1 =  (analogRead(ldr_1_in)) ;
  float ldr2 =  (analogRead(ldr_2_in)) ;
  float ldr3 =  (analogRead(ldr_3_in)) ;
  float ldr4 =  (analogRead(ldr_4_in)) ;

  Serial.print("Nomralised Ambient Light Intensity: ") ;
  Serial.println((ldr1 + ldr2 + ldr3 + ldr4 + ldrref) / 500) ;

/*Serial.println("Obtained Intensisties:") ;
  Serial.println(ldrref) ;
  Serial.println(ldr1) ;
  Serial.println(ldr2) ;
  Serial.println(ldr3) ;
  Serial.println(ldr4) ;*/

  if ( (ldr1 + ldr2 + ldr3 + ldr4 + ldrref) / 500 < 0.70) {
    Serial.println("Overcast!! ") ;
    error[0] = -1 ; // -1 at index 0  means overcast!!!
    return error ;
  }
  
  else {
    error[0] = ldrref ;
    error[1] = (ldr1 - ldr3) / ldrref ; //For Actuator1
    error[2] = (ldr2 - ldr4) / ldrref ; //For Actuator2
    return error ;
  }
  
} 



void loop() {
  // put your main code here, to run repeatedly:
  //init actuation commands
  int actuation_commands[2] = { 0, 0 } ;

  Serial.println("CONTROL LOOP************");
  Serial.println("PANEL ADJUSTMENTS STARTING (3 MINUTES LOOP)!") ;
  int command[2] ;
  
  // 1. Astronomical method based adjustmensts:
  if ( astronomical_method_reqd ) {
    Serial.print("**********Astronomical Adjustments...") ;
    //array of commands obtained from rtc+magnetometer in terms of voltage derived from theta reqd
    command[0] =  pid_controller ( astro_commands()[0], Kp_1, Ki_1, Kd_1 ) ; //for act 1
    command[1] =  pid_controller ( astro_commands()[1], Kp_2, Ki_2, Kd_2 ) ; //for act 2
    execute_actuator( command ) ;
    Serial.println("Done!") ;
  }

  // 2. Control Loop: LDR input
  int reference_time = millis() ;
  Serial.println("**********LDR Control Loop Starting...") ;
  
  while ( millis() - reference_time < panel_adjustment_time ) { // 3min real time LDR based actuator control loop
    
    int *error = LDR_errors() ;
    if ( error[0] == -1 ) {
      //****actuate for basic flat position of solar panel, acheived using pitch and roll readings
      continue ;
    }
    
    Serial.print( "Obtained Errors: " ) ; Serial.print( error[1] ) ; Serial.print( "  |  " ) ; Serial.println( error[2] ) ;
    //TODO: Switch betwen various control methods
    
    command[0] =  pid_controller ( error[1], Kp_1, Ki_1, Kd_1 ) ; //for act 1
    command[1] =  pid_controller ( error[2], Kp_2, Ki_2, Kd_2 ) ; //for act 2
    Serial.println("Controlling and Acutating...") ;
    Serial.print( "Generated Commands: " ) ; Serial.print( command[0] ) ; Serial.print( "  |  " ) ; Serial.println( command[1] ) ;
    execute_actuator( command ) ;
    //delay( iteration_time ) ;

  }

  //Sleep time: Interupted when substantial changes in yaw(heading)
  Serial.println("SLEEP TIME STARTING !") ;
  reference_time = millis() ;
  int reference_heading = magnetometer("yaw") ;
  while ( ( millis() - reference_time ) < sleep_time ) { // 15min battery recharge and sleep time
    if ( heading_interrupt_reqd && ( abs( magnetometer("yaw") - reference_heading ) > 45 ) ) { // 45deg heading threshold
      Serial.println("<<< Panel has been moved. Stabilisation Disrupted! >>>") ;
      break ;
    }
  }

  //End of loop
}
