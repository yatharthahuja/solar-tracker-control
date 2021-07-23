//LDR Array input ports:
const int ref_ldr_in = A3 ;
const int ldr_1_in = A0 ;
const int ldr_2_in = A1 ;
const int ldr_3_in = A2 ;
const int ldr_4_in = A4 ;
const int base_port = 7 ;

void setup() {
  // put your setup code here, to run once:
 pinMode( ref_ldr_in, INPUT) ;
  pinMode( ldr_1_in, INPUT) ;
  pinMode( ldr_2_in, INPUT) ;
  pinMode( ldr_3_in, INPUT) ;
  pinMode( ldr_4_in, INPUT) ;
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
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
  
}
