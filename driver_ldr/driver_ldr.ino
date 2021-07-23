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

//LDR Array input ports:
const int ref_ldr_in = A0 ;
const int ldr_1_in = A3 ;
const int ldr_2_in = A1 ;
const int ldr_3_in = A2 ;
const int ldr_4_in = A4 ;

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

  //LDR ports:
  pinMode( ref_ldr_in, INPUT) ;
  pinMode( ldr_1_in, INPUT) ;
  pinMode( ldr_2_in, INPUT) ;
  pinMode( ldr_3_in, INPUT) ;
  pinMode( ldr_4_in, INPUT) ;
  
  Serial.println("Done!") ;

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
  delay(1000) ; //mandatory delay for adjustment and iteration time
}

int pid1(float err){
  if (abs(err)<70){
    return 0;
  }
  int t = 2;//seconds
  int kp = 15;int kd = 7;
  int cmd = err*(kp + kd/t);
  return cmd;
}

int pid2(float err){
  if (abs(err)<70){
    return 0;
  }
  int t = 2;//seconds
  int kp = 20;int kd = 30;
  int cmd = err*(kp + kd/t);
  return cmd;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Starting...");
  Serial.print("Ref: ");
  Serial.println(analogRead(ref_ldr_in));
  Serial.print("LDR 1: ");
  Serial.println(analogRead(ldr_1_in));
  Serial.print("LDR 2: ");
  Serial.println(analogRead(ldr_2_in));
  Serial.print("LDR 3: ");
  Serial.println(analogRead(ldr_3_in));
  Serial.print("LDR 4: ");
  Serial.println(analogRead(ldr_4_in));
  float err_1 = ((analogRead(ldr_1_in) - analogRead(ldr_3_in)));//analogRead(ref_ldr_in) ;
  Serial.print("err_1: ");
  Serial.println(err_1);
  float err_2 = ((analogRead(ldr_2_in) - analogRead(ldr_4_in)));//analogRead(ref_ldr_in);
  Serial.print("err_2: ");
  Serial.println(err_2);
  int cmd_1 = pid1(err_1);
  int cmd_2 = pid1(err_2);  
  int command[2] = { cmd_1, cmd_2 } ;
  execute_actuator(command) ;
  //delay(3000) ;
}
