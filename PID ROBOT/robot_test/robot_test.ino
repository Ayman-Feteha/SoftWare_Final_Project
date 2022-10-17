/*
      --documentation--
*/

/*
#include
*/

//pins decleration
// Motor A connections
int enA = 9;
int in1 = 6;
int in2 = 7;
int enc2 = 3; //the encoder connected with wheel of motor A
// Motor B connections
int enB= 10;
int in3= 4;
int in4= 5;
int enc1 = 2; //the encoder connected with wheel of motor B

// Global Variables decleration
int N_tics1,N_tics2;  //the number of tics counted from encoder 1 and encoder 2
int diff; //the difference between the number of tics counted from encoder1 and encoder 2
int state,tics1,tics2;  //the state given from serial communication to control the robot motion, tics1 & tics2 declare the state of encoder (1 or 0)

void setup() {
  
  Serial.begin(9600);
    
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  for (int i=4;i<8;i++){
    pinMode(i,OUTPUT);
    }
  // set encoders pins to inputs
  pinMode(enc1, INPUT);
  pinMode(enc2, INPUT);
  
  // Turn off motors - Initial state
  for (int i=4;i<8;i++){
    digitalWrite(i,LOW);
    }
  // attach encoders to interrupt pins so that the N_tics will be increased as soon as the encoder reads (1) not if it reads (1) during certain time in the loop
  attachInterrupt(digitalPinToInterrupt(enc1), Inc_tics1 , RISING);
  attachInterrupt(digitalPinToInterrupt(enc2), Inc_tics1 , RISING);
    
}

void loop() {

}

//Function of interrupt to increase the tics number of encoder of motor B
void Inc_tics1(){
  N_tics1+=1;
  }

//Function of interrupt to increase the tics number of encoder of motor A
void Inc_tics2(){
  N_tics2+=1;
  }
  
//Function to limit the tics number of encoders to avoid exceeding the storage limit
void limit_counter(){
  N_tics1-=1000;
  N_tics2-=1000;
  }

//Function to reset the tics number of encoders if the pilot stops the robot 
void reset_counter{
  N_tics1=N_tics2=0;
  }
  
