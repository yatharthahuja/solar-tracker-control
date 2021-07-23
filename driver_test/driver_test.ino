//Motor Driver: Actuator1-6inch
const int LEN_1 = 2;
const int REN_1 = 4;
const int LPWM_1 = 3;
const int RPWM_1 = 5;

//Motor Driver: Actuator2
const int LEN_2 = 9;
const int REN_2 = 8;
const int LPWM_2 = 6;
const int RPWM_2 = 7;

void setup(){
  Serial.begin(9600) ;

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
  
  digitalWrite(REN_1,HIGH);
  digitalWrite(REN_2,HIGH);
  digitalWrite(LEN_1,HIGH);
  digitalWrite(LEN_2,HIGH);
}

void execute_actuator( int command[] ) { //PWM speed to be adjusted as per voltage available in the battery and iterataion time
  //Index-> 0 for Act1-6inch; 1 fror Act2-8inch
 
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
    digitalWrite(LEN_1,HIGH);
    digitalWrite(REN_1,HIGH);
    analogWrite(RPWM_1,0);
    analogWrite(LPWM_1,255);
    Serial.print("Command executed: (Act1) ");
    Serial.println(command[0]);
    delay(1000);
  }

  if ( command[1]>0 ) { //Actuator2
    digitalWrite(REN_2,HIGH);
    digitalWrite(REN_2,HIGH);
    analogWrite(RPWM_2,255);
    analogWrite(LPWM_2,0);
    Serial.print("Command executed: (Act2) ");
    Serial.println(command[1]);
    delay(1000);
  }
  if ( command[1]<0 ) { //Actuator2
    digitalWrite(LEN_2,HIGH);
    digitalWrite(LEN_2,HIGH);
    analogWrite(RPWM_2,0);
    analogWrite(LPWM_2,255);
    Serial.print("Command executed: (Act2) ");
    Serial.println(command[1]);
    delay(1000);
  }
  //Serial.print( "Executed Commands: " ) ; Serial.print( command[0] ) ; Serial.print( "  |  " ) ; Serial.println( command[1] ) ;
  delay(1000) ; //mandatory delay for adjustment and iteration time

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Starting...");
  digitalWrite(REN_1,HIGH);
  digitalWrite(REN_2,HIGH);
  digitalWrite(LEN_1,HIGH);
  digitalWrite(LEN_2,HIGH);
  int command[2] = { -255, -255 } ;
  execute_actuator(command) ;
  delay(1000);
  int command1[2] = { 255, 255 } ;
  execute_actuator(command1) ;
  delay(1000);
  int command2[2] = { -255,  -255 } ;
  execute_actuator(command2) ;
  delay(1000);
}
