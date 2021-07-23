//SYNOPSIS:
//This code manages alternations between the 2 batteries acquired for Dual-Ais tracker system 
//One Battery provides power to the load and the other is charged meanwhile through solar input
//Thus constant sustainable supply is managed
//Voltages are compared and switchings are made accordingly by Arduino UNO micrcontroller
//All voltages reading multiplied by 100 to avoid float comparisons in Arduino
//
//          **INDEXING**
//
//  __^_______^__
//  |           |      Switch to Source:  Switch 1
//  | Battery 1 |      Switch to Load:    Switch 3
//  |___________|      Voltage Variable:  V1
//
//  __^_______^__
//  |           |      Switch to Source:  Switch 2
//  | Battery 2 |      Switch to Load:    Switch 4
//  |___________|      Voltage Variable:  V2
//
//
//FOR REFERENCE: panel_side_ckt (Proteus Simulation File/Diagram)
//


//Declarations

const double reqd_battery_level = 14.4*100 ;//sustainable charged battery voltage level
const int switching_time = 100 ;//switching time for transistors in micro seconds

int last_battery_charged_index = 1 ; //data log variable carried to keep track of last battery charged

const int volt_in_1 = 4 ;
const int volt_in_2 = 3 ;

double V1, V2 ;

const int switch_out_1 = 11 ;
const int switch_out_2 = 10 ;
const int switch_out_3 = 9 ;
const int switch_out_4 = 6 ;

//Testing Variable
const bool testing = false ;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600) ;
  Serial.begin(9600) ;
    
  if ( testing ){ //pseudo values
    V1 =1220;
    V2 =1200;
  }

  pinMode( volt_in_1, INPUT) ;
  pinMode( volt_in_2, INPUT) ;
    
  pinMode( switch_out_1, OUTPUT) ;
  pinMode( switch_out_2, OUTPUT) ;
  pinMode( switch_out_3, OUTPUT) ;
  pinMode( switch_out_4, OUTPUT) ;

  digitalWrite(switch_out_1, LOW) ;
  digitalWrite(switch_out_2, LOW) ;
  digitalWrite(switch_out_3, LOW) ;
  digitalWrite(switch_out_4, LOW) ;

}

void switch_status( char* str1, char* str2, char* str3, char* str4 ){//to generate testing plot

  Serial.print("Switch Status 1: ") ;
  Serial.println(str1) ;
  Serial.print("Switch Status 2: ") ;
  Serial.println(str2) ;
  Serial.print("Switch Status 3: ") ;
  Serial.println(str3) ;
  Serial.print("Switch Status 4: ") ;
  Serial.println(str4) ;
  Serial.println("*****************") ;
  Serial.println(" ") ;

}

void plot( int V1, int V2 ){
  Serial.print("| V1:");Serial.print(V1);Serial.print("  |  V2:");Serial.print(V2);Serial.println(" |");
}

void loop() {
  // put your main code here, to run repeatedly:  
  
  //To init plot arguments
  char str1[4] = "LOW";
  char str2[4] = "LOW";
  char str3[4] = "LOW";
  char str4[4] = "LOW";
  
  
  if ( !testing ){
    
    V1 = analogRead(volt_in_1)*100 ;
    V2 = analogRead(volt_in_2)*100 ;  
  
  }
  
  if ( (V1 < reqd_battery_level) || (V2 < reqd_battery_level) ){ //Atleast one of the batteries requires charging
  
    if ( V2 > V1 ) { // charging Battery 1
      
      digitalWrite(switch_out_1, HIGH) ;   //str1 = "HIGH" ;
      digitalWrite(switch_out_4, HIGH) ;   //str4 = "HIGH" ;
      delay(switching_time) ;
      digitalWrite(switch_out_1, LOW) ;
      digitalWrite(switch_out_4, LOW) ;    

      last_battery_charged_index = 1 ;
      
    }
      
    else if ( V1 > V2 ) { // charging Battery 2
      
      digitalWrite(switch_out_2, HIGH) ;   //str2 = "HIGH" ;
      digitalWrite(switch_out_3, HIGH) ;   //str3 = "HIGH" ;
      delay(switching_time) ;
      digitalWrite(switch_out_2, LOW) ;   
      digitalWrite(switch_out_3, LOW) ;   

      last_battery_charged_index = 2 ;
      
    }

    if ( testing ){
      
      switch(last_battery_charged_index){
        
        case 1 :
          V1 += 20 ; //simulating charging for testing of code
          Serial.print("V1 charged!....") ;
          break ;
          
        case 2:
          V2 += 20 ; //simulating charging for testing of code
          Serial.print("V2 charged!....") ;
          break ;
      
      }
    
    }
    
  }

  plot( V1, V2) ;
  //switch_status( str1, str2, str3, str4 ) ;
  delay(2000) ;
  
}
