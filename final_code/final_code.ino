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
#include "trapmf.h" // for FLC 
#include "printFloat.h" // for FLC


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
const int device = 1 ;
const int rtcaddress = 0x68 ;
const int compassaddress = 0x3D ;
//const char *months[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" } ;
const int latitude = 45 ; // Enter latitude of the location in degrees, roughly
const int days_in_month_normal[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 } ;
//const int days_in_month_leap[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 } ;
HMC5883L_Simple Compass ;
tmElements_t tm ;
/* The I2C address of the module */
#define HMC5803L_Address 0x1E
/* Register address for the X Y and Z data */
#define X 3
#define Y 7
#define Z 5

//Assumed constant supply Voltage
const int Vbattery = 10 ;

//whether to involve astronomical method in the control:
const bool astronomical_method_reqd = false ;

//whether to involve heading correction method in the control:
const bool heading_interrupt_reqd = false ;

//Control method
const int control_method = 1 ; //Select Option: < 1 for "PID" >, < 2 for "PIID" >, < 3 for "Flat" >, < 4 for "Fuzzy-ANN" >

//Time master loop distribution settings ( in microseconds )
const unsigned long sleep_time = 390000 ; //6.5 mins //only sudden change in heading(yaw) causes tracking during this
const unsigned long panel_adjustment_time = 60000 ;//1min //timely tracking
const int iteration_time = 2000 ;//2sec //for execution of one control loop

// Proportional Constants relating Angular velocity to voltage input for each actuator(0-> 8inch, 1-.6inch) 
const int voltage_to_omega[2] = { 4.95, 3.37 } ; // V = k * w ; k described here

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

/*
//PID constans(for each actuator)//ZN Method:
//Actuator-1:
const float Kp_1 = 19.33 ;
const float Kd_1 = 29.67 ;
const float Ki_1 = 8.51 ;
//Actuator-2:
const float Kp_2 = 15.82 ;
const float Kd_2 = 8.51 ;
const float Ki_2 = 7.36 ;
*/

//PID constans(for each actuator)//GA Method:
//Actuator-1:
const float Kp_1 = 476.34 ;
const float Kd_1 = 5.06 ;
const float Ki_1 = 81.45 ;
//Actuator-2:
const float Kp_2 = 498.72 ;
const float Kd_2 = 23.08 ;
const float Ki_2 = 15.19 ;

//PID constans(for each actuator)//GA Method:
//Actuator-1:
const float Flat_Kp_1 = 760490.72 ;
const float Flat_Kd_1 = 355.73 ;//-355.73
const float Flat_Ki_1 = 0 ;
//Actuator-2:
const float Flat_Kp_2 = 334295.75 ;
const float Flat_Kd_2 = 355.73 ;//-355.73
const float Flat_Ki_2 = 0 ;


/*
//PID constans(for each actuator)//SA Method:
//Actuator-1:
const float Kp_1 = 151.73 ;
const float Kd_1 = 117.82 ;
const float Ki_1 = 59.16 ;
//Actuator-2:
const float Kp_2 = 231.86 ;
const float Kd_2 = 156.19 ;
const float Ki_2 = 32.41 ;
*/


// Cummulative error for PID, 0 index for 8inch & 1 index for 6inch
int cummulative_error[2] = { 0, 0 } ;

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
  Init_HMC5803L() ;//Initialise Magnetometer module
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

  //For PID:
  //int cummulative_error = 0 ;
  
  Serial.println("Done!") ;

}


//Function to provide normalised LDR errors:
int* LDR_errors() {

  int error[3] = { 0, 0, 0 } ; //Index assignment: 0->ldrref, 1->(ldr1-ldr3), 2->(ldr2-ldr4)
  float ldrref = (analogRead(ref_ldr_in)) ;
  float ldr1 =  (analogRead(ldr_1_in)) ;
  float ldr2 =  (analogRead(ldr_2_in)) ;
  float ldr3 =  (analogRead(ldr_3_in)) ;
  float ldr4 =  (analogRead(ldr_4_in)) ;

  Serial.print("Normalised Ambient Light Intensity: ") ;
  Serial.println((ldr1 + ldr2 + ldr3 + ldr4 + ldrref) / 500) ;

/*Serial.println("Obtained Intensisties:") ;
  Serial.println(ldrref) ;
  Serial.println(ldr1) ;
  Serial.println(ldr2) ;
  Serial.println(ldr3) ;
  Serial.println(ldr4) ;*/
  Serial.println(error[1]);
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

void execute_actuator( int command[] ) { //PWM speed to be adjusted as per voltage available in the battery and iterataion time
  //Index-> 0 for Act1-6inch; 1 fror Act2-8inch
  digitalWrite(REN_1,HIGH);
  digitalWrite(LEN_1,HIGH);
  digitalWrite(REN_2,HIGH);
  digitalWrite(LEN_2,HIGH);
 
  //command[0] = ( command[0]/*Vbattery*/ )*255 ;
  //command[1] = ( command[1]/*Vbattery*/ )*255 ;
 
  if ( command[0]>255){//255 ){
    command[0] = 255 ;
  }
  if ( command[0]<-255){//-255 ){
    command[0] = -255 ;
  }
  if ( command[1]>255){//255 ){
    command[1] = 255 ;
  }
  if ( command[1]<-255){//-255 ){
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
  delay(iteration_time) ; //mandatory delay for adjustment and iteration time

  digitalWrite(REN_1,LOW);
  digitalWrite(LEN_1,LOW);
  digitalWrite(REN_2,LOW);
  digitalWrite(LEN_2,LOW);
}

/* This function will initialise the module and only needs to be run once
   after the module is first powered up or reset */
void Init_HMC5803L( void ) {
  /* Set the module to 8x averaging and 15Hz measurement rate */
  Wire.beginTransmission(HMC5803L_Address);
  Wire.write(0x00);
  Wire.write(0x70);
          
  /* Set a gain of 5 */
  Wire.write(0x01);
  Wire.write(0xA0);
  Wire.endTransmission();
}


/* This function will read once from one of the 3 axis data registers
and return the 16 bit signed result. */
int HMC5803L_Read(byte Axis)
{
  int Result;
  
  /* Initiate a single measurement */
  Wire.beginTransmission(HMC5803L_Address);
  Wire.write(0x02);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(6);
  
  /* Move modules the resiger pointer to one of the axis data registers */
  Wire.beginTransmission(HMC5803L_Address);
  Wire.write(Axis);
  Wire.endTransmission();
   
  /* Read the data from registers (there are two 8 bit registers for each axis) */  
  Wire.requestFrom(HMC5803L_Address, 2);
  Result = Wire.read() << 8;
  Result |= Wire.read();

  return Result;
}

//Function to return magnetometer parameter readings as per demand:
int magnetometer ( char* parameter ) {

  
 /* 
  Wire.beginTransmission(compassaddress) ; //Tell the HMC5883L where to begin reading data
  //Wire.send(0x03) ; //select register 3, X MSB register
  Wire.endTransmission() ;
  */
  int yaw, pitch, roll ;
  pitch = HMC5803L_Read(X);
  yaw = HMC5803L_Read(Y);
  roll = HMC5803L_Read(Z);
  
  /*int heading = Compass.GetHeadingDegrees() ;
 
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
  */
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
    //int Second = tm.Second ;
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

  int rtc_output[2] ;
  *rtc_output = rtc() ;

  //Day of the year(out of 365) - from RTC
  int day_of_year = rtc_output[0] ;
  //Hour of the day - from RTC
  int hour_of_day = rtc_output[1] ;
  
  //float zenith_angle = 0 ;
  //float incidence_angle = 0 ;//Angle to be achieved in N-S direction. Angle between surface normal and solar radiation
  float clock_angle = ( hour_of_day-12 )*15 ;//Angle to be achieved in E-W direction

  //float azimuthal_angle = 0 ;
  float declination_angle = 23.5 * sin( 360*(284+day_of_year)/365 ) ;
    
  //Heading(yaw) in degrees with respect to true North - from Magnetometer
  int heading = magnetometer( "yaw" ) ;

  //Function to provide theta by breaking down into respective components for each actuator:
  int theta[2] ;//thetas to be executed respectively: theta[0]->Actuator1, theta[1]->Actuator2
  
  //Formulae for the 2 angles(theta below) here ; initial factor inclusded for conversion to PWM voltage for actuation command  
  // TBD: convert theta angle values to realtive voltage values with respect to the pre fixed actuation time of 2s
  // theta/2 = w = k V // V / Vbattery = command / 255 // theta * (255 * 1000) / ( iteration_time * k * Vbattery)  
  theta[0] = ( (255*1000) / ( iteration_time * voltage_to_omega[0] * Vbattery ) ) * (acos( ( cos(latitude)*cos(clock_angle)*cos(declination_angle) ) + ( sin(latitude)*sin(declination_angle) ) ) - magnetometer( "roll" ))/magnetometer( "roll" ) ;//In N-S Direction With respect to vertical
  theta[1] = ( (255*1000) / ( iteration_time * voltage_to_omega[1] * Vbattery ) ) * (clock_angle-magnetometer( "pitch" ))/magnetometer( "pitch" ) ;//in E-W direction with respect to the vertical 

  //theta = componented_theta( theta ) ; //***********TBD
  //Actuation commands obtained:
  //return theta_to_voltage( theta ) ;
  Serial.print("Theta error related commands:  ");
  Serial.print(theta[0]);
  Serial.print(" | ");
  Serial.println(theta[0]);
  return theta ;

}

int pid_controller ( int error, int Kp, int Ki, int Kd, int cummulative_error_index ) {
  float cmd = ( error * (Kp + Kd / iteration_time) ) + ( Ki * iteration_time * cummulative_error[cummulative_error_index] ) ;
  cummulative_error[cummulative_error_index] += error ;
  return cmd ;
}

int flc_controller( int error ){
  
  int error_diff = error/iteration_time ; 
  float out = 0 ; 
  float out_NL[5] = {0};
  float out_NS[7] = {0};
  float out_ZE[1] = {0};
  float out_PS[5] = {0};
  float out_PL[7] = {0};

  float error_NL = 0;
  float error_NS = 0;
  float error_ZE = 0;
  float error_PS = 0;
  float error_PL = 0;
  float error_diff_NL = 0;
  float error_diff_NS = 0;
  float error_diff_ZE = 0;
  float error_diff_PS = 0;
  float error_diff_PL = 0;
  float out_NL_max = 0;
  float out_NS_max = 0;
  float out_ZE_max = 0;
  float out_PS_max = 0;
  float out_PL_max = 0;

  while(1)  {

    // evaluate the membership functions at the measured error
    error_NL = trapmf(error, -150,  -100,  -50);
    error_NS = trapmf(error, -60, -40, -20);
    error_ZE = trapmf(error, -30, 0, 30);
    error_PS = trapmf(error, 20, 40, 60);
    error_PL = trapmf(error, 50, 100, 150);

    // evaluate the membership functions at the measured error differential
    error_diff_NL = trapmf(error_diff, -30,  -20,  -10);
    error_diff_NS = trapmf(error_diff, -20,  -10,  0);
    error_diff_ZE = trapmf(error_diff, -10,  0,  10);
    error_diff_PS = trapmf(error_diff, 0,  10,  20);
    error_diff_PL = trapmf(error_diff, 10,  20,  30);

    //row1 (of rule table)
    out_PL[0] = AND(error_NL, error_diff_NL);
    out_PS[0] = AND(error_NL, error_diff_NS);
    out_NL[0] = AND(error_NL, error_diff_ZE);
    out_NS[0] = AND(error_NL, error_diff_PS);
    out_NS[1] = AND(error_NL, error_diff_PL);
    
    //row2 (of rule table)
    out_PS[1] = AND(error_NS, error_diff_NL);
    out_PS[2] = AND(error_NS, error_diff_NS);
    out_NL[1] = AND(error_NS, error_diff_ZE);
    out_NS[2] = AND(error_NS, error_diff_PS);
    out_NS[3] = AND(error_NS, error_diff_PL);    
    
    //row3 (of rule table)
    out_NS[4] = AND(error_ZE, error_diff_NL);
    out_NS[5] = AND(error_ZE, error_diff_NS);
    out_ZE[0] = AND(error_ZE, error_diff_ZE);
    out_PL[1] = AND(error_ZE, error_diff_PS);
    out_PL[2] = AND(error_ZE, error_diff_PL);
    
    //row4 (of rule table)
    out_NS[6] = AND(error_PS, error_diff_NL);
    out_PL[3] = AND(error_PS, error_diff_NS);
    out_PS[3] = AND(error_PS, error_diff_ZE);
    out_NL[2] = AND(error_PS, error_diff_PS);
    out_PL[4] = AND(error_PS, error_diff_PL);
    
    //row5 (of rule table)
    out_NL[3] = AND(error_PL, error_diff_NL);
    out_NL[4] = AND(error_PL, error_diff_NS);
    out_PL[5] = AND(error_PL, error_diff_ZE);
    out_PS[4] = AND(error_PL, error_diff_PS);
    out_PL[6] = AND(error_PL, error_diff_PL);
    
    // now that we have anded the D.o.T. we need to find the OR(max) of each
    out_NL_max = ArrMax(out_NL, 5);
    out_NS_max = ArrMax(out_NS, 7);
    out_ZE_max = ArrMax(out_ZE, 1);
    out_PS_max = ArrMax(out_PS, 5);
    out_PL_max = ArrMax(out_PL, 7);

    // Calculate the weighted average using the crisp consequent
    out = (out_NL_max*(-255) + out_NS_max*(-155) + out_ZE_max*0 + out_PS_max*155 + out_PL_max*255);
    out /= (out_NL_max+ out_NS_max+ out_ZE_max+ out_PS_max+ out_PL_max);
  }
  return out ; 
}

int flat_controller ( int error, int Kp, int Ki, int Kd ) {
  float cmd = ( error * (Kp + Kd / iteration_time) ) ;
  //cummulative_error[cummulative_error_index] += error ;
  return cmd ;
}

void loop() {
  // put your main code here, to run repeatedly:
  //init actuation commands
  int actuation_commands[2] = { 0, 0 } ; 
  
  Serial.println("CONTROL LOOP************");
  Serial.println("PANEL ADJUSTMENTS STARTING (1 MINUTE LOOP)!") ;
  int command[2] ;
  
  // 1. Astronomical method based adjustmensts:
  if ( astronomical_method_reqd ) {
    Serial.print("**********Astronomical Adjustments...") ;
    //array of commands obtained from rtc+magnetometer in terms of voltage derived from theta reqd
    *command = astro_commands() ;
    Serial.print( "Generated Commands: " ) ; Serial.print( command[0] ) ; Serial.print( "  |  " ) ; Serial.println( command[1] ) ;
    execute_actuator( command ) ;
    Serial.println("Done!") ;
  }
  
  // 2. Control Loop: LDR input
  int reference_time = millis() ;
  Serial.println("**********LDR Control Loop Starting...") ;
  
  while ( millis() - reference_time < panel_adjustment_time ) { // 3min real time LDR based actuator control loop
 
    int error[3] = { 0, 0, 0 } ; //Index assignment: 0->ldrref, 1->(ldr1-ldr3), 2->(ldr2-ldr4)
    float ldrref = (analogRead(ref_ldr_in)) ;
    float ldr1 =  (analogRead(ldr_1_in)) ;
    float ldr2 =  (analogRead(ldr_2_in)) ;
    float ldr3 =  (analogRead(ldr_3_in)) ;
    float ldr4 =  (analogRead(ldr_4_in)) ;
     
    Serial.print("Nomralised Ambient Light Intensity: ") ;
    Serial.println((ldr1 + ldr2 + ldr3 + ldr4 + ldrref) / 5) ;
  /*   
    Serial.println("Obtained Intensisties:") ;
    Serial.println(ldrref) ;
    Serial.println(ldr1) ;
    Serial.println(ldr2) ;
    Serial.println(ldr3) ;
    Serial.println(ldr4) ;
  */  
    if ( (ldr1 + ldr2 + ldr3 + ldr4 + ldrref) / 500 < 0.70) {
      Serial.println("Overcast!! ") ;
      error[0] = -1 ; // -1 at index 0  means overcast!!!
      //return error ;
    }
  
    else {
      error[0] = ldrref ;
      error[1] = (ldr1 - ldr3)*255 / ldrref ; //For Actuator1
      error[2] = (ldr2 - ldr4)*255 / ldrref ; //For Actuator2
      //return error ;
    }
    //int *error = LDR_errors() ;
    //Serial.println(error[1]);
    if ( error[0] == -1 ) {
      //****actuate for basic flat position of solar panel, acheived using pitch and roll readings
      continue ;
    }
    
    Serial.print( "Obtained Errors: " ) ; Serial.print( error[1] ) ; Serial.print( "  |  " ) ; Serial.println( error[2] ) ;
    //TODO: Switch betwen various control methods
    
    //command[0] =  pid_controller ( error[1], Kp_1, Ki_1, Kd_1, 0 ) ; // for act 1
    command[0] =  flat_controller ( error[1], Flat_Kp_1, Flat_Ki_1, Flat_Kd_1 ) ; // for act 1
    //if( abs(error[0]) < 20 ){ // thresholding
    //  command[0] = 0;
    //}
    command[1] =  flat_controller ( error[2], Flat_Kp_2, Flat_Ki_2, Flat_Kd_2 ) ; // for act 2
    //if( abs(error[1]) < 20 ){ // thresholding
    //  command[1] = 0;
    //}
    Serial.println("Controlling and Acutating...") ;
    Serial.print( "Generated Commands: " ) ; Serial.print( command[0] ) ; Serial.print( "  |  " ) ; Serial.println( command[1] ) ;
    execute_actuator( command ) ;
  
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
