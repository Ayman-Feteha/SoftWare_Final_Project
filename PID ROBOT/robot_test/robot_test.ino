int enA = 9;
int in1 = 2;
int in2 = 3;
// Motor B connections
int enB= 10;
int in3= 4;
int in4= 5;
int enc1 = 6;
int enc2 = 7;
int diff;
int N_tics1,N_tics2;
int state,tics1,tics2;
float P_value,d_value;
void setup() {
  // Set all the motor control pins to outputs
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enc1, INPUT);
  pinMode(enc2, INPUT);
  
  // Turn off motors - Initial state
 /* digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB,100 );
  analogWrite(enA,100);*/
}

void loop() {
 tics1=digitalRead(enc1);
 tics2=digitalRead(enc2);
  if (Serial.available()>0){
     state=Serial.read();
    }
  if (state=='a'){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enB,80);
      analogWrite(enA,80);
       if (tics1==1){
  N_tics1+=1;
  }
 if (tics2==1){
  N_tics2+=1;
  }
   Serial.println(N_tics1);
  
 Serial.println(N_tics2);
if (N_tics1>=N_tics2){
  diff = N_tics1-N_tics2;
  P_value=diff*5;
  analogWrite(enB,80+P_value);
  }
  else{
    diff=N_tics2-N_tics1;
    d_value=diff*1.5;
    analogWrite(enB,80+P_value-d_value);
    }
  
    }
    else if(state=='s'){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB,0 );
  analogWrite(enA,0);
      }
     else if(state=='d'){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB,70 );
  analogWrite(enA,70);
      }
      

 //Serial.println(tics1);
  
 //Serial.println(tics2);
 Serial.println("///////////////////");

}

// This function lets you control spinning direction of motors
/*void directionControl() {
  // Set motors to maximum speed
  // For PWM maximum possible values are 0 to 255
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  // Turn on motor A & B
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(2000);
  
  // Now change motor directions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(2000);
  
  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}*/
/*
// This function lets you control speed of the motors
void speedControl() {
  // Turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  }
  
  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  }
  
  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}*/
