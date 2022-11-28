/*
      This code implements a control on a two wheel differential robot the control is mainly implemented on the speed of the robot in rpm and the angle of the robot.
      The speed control is done by the optical encoder attached to each wheel of the robot. The encoder gives HIGH value each time the ir sensor catches a light
      through the encoder, this HIGH value that from the sensor is used as a tick to represent a step done by the wheel then by counting the number of ticks done by the
      wheel and measuring the diameter of the wheel then we can know the displacement the robot has made then we can calculate from it the speed in of the wheel in rpm.
      The angle of the robot is controlled by a PID controller that takes an input as a yaw angle from the imu(mpu-9255) and calcualtes the current error from the desired setpoint,
      then the PID output added or minused from the motor speeds depending on the direction of the robot.       
*/


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/////////////// bn00
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);



// ################### MOTOR DRIVER PINS DECLARATION #####################
// Right Motor connections
int enR = 9; // pwm pin 
int in1 = 6; // direction pin 1
int in2 = 7; // direction pin 2
int enc2 = 3; //the encoder connected with wheel of motor A
// Left Motor connections
int enL= 10; // pwm pin 
int in3= 4;  // direction pin 3
int in4= 5;  // direction pin 4
int enc1 = 2; //the encoder connected with wheel of motor B



// ################# WHEEL  GLOBAL VARIABLES DECLARATION ####################
int N_tics1_curr, N_tics2_curr;  //the number of tics counted from encoder 1 and encoder 2
int diff1 ,diff2; //the difference between the number of tics counted from encoder1 and encoder 2
int state,tics1,tics2;  //the state given from serial communication to control the robot motion, tics1 & tics2 declare the state of encoder (1 or 0)




// ############### PID VARIABLES #############
int g_yaw = 0;          // The current yaw angle from the imu   
int setpoint = 0;
float error = 0;        // current error
float prev_error = 0;   // previous error
int prevMeasurment = 0;

float Kp = 2.0;
float Ki = 0.29;
float Kd = 0.9;


;

float tau = 0.02;
float proportional = 0; // proportional error 
float integral = 0;     // integral error
float derivative = 0;   // derivative error
float p_term = 0; 
float i_term = 0;
float d_term = 0;
int i_max = 80;         // I term max limit
int i_min = -100;        // I term min limit
float out_max = 100;     // output max limit
float out_min = -100;    // output min limit
float pid_out = 0;      // PID output
long prevT = 0;        
float dt = 0;           // difference in time

float T = 0.005;


// ################ SPEED CONTROL VARIABLES #################
int rpm = 100; // rpm speed  

int pwm_r = round((100 * rpm) / 345.02);  // pwm value applied on the right motor
int pwm_l = round((100 * rpm) / 310);     // pwm value applied on the left motor
//float out_max = pwm_r;     // output max limit
//float out_min = pwm_l;    // output min limit




void setup() {

  
  Wire.begin();
  Serial.begin(115200);

/////////////////////////////////
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }


    bno.setExtCrystalUse(true);
    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
///////////////////////////////
  
    
  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  
  // setting driver direction pins
  for (int i=4;i<8;i++){
    pinMode(i,OUTPUT);
    }
  // set encoders pins to inputs
  pinMode(enc1, INPUT);
  pinMode(enc2, INPUT);
  
  
  // attaching interrupt pins to the encoder to count each time the encoder gives HIGH
//  attachInterrupt(digitalPinToInterrupt(enc1), Inc_tics1 , RISING);
//  attachInterrupt(digitalPinToInterrupt(enc2), Inc_tics2 , RISING);

  delay(3000);
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // This loop takes imu readings first before initializing a setpoint because first readings are not accurate
   setpoint =euler.x();
  Serial.print("Setpoint :");
  Serial.println(setpoint);

}

void loop() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /////////////////
// Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.println(euler.x());


/////////////////////////////

   
    float returned_out;  // This variable saves the output returnd from the PID  function
    long currT = micros(); // saving the current time 
    dt = (currT-prevT)/1.0e-6; // calculating the difference time 
    prevT = currT;

   

    returned_out = pid(euler.x()); 

    
    /* 
     * checking if the robot is moved to the right then the opposing movement to correct the error  
     * will be to the left by speeding up the right wheel and decreasing the left wheel speed
     */ 
    
    
      /*
       * The direction of the wheel is set to move forward
       */
       if(returned_out > 0){
        
       
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      
      analogWrite(enR, abs(returned_out )); // increasing right wheel speed
      analogWrite(enL, abs(returned_out )); // decreasing left wheel speed   
       }

    /*
     * checking if the robot is moved to the left then the opposing movement to correct the error  
     * will be to the right by speeding up the left wheel and decreasing the right wheel speed
     */ 
    else
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      
      analogWrite(enR, abs(returned_out)); // decreasing right wheel speed
      analogWrite(enL, abs(returned_out)); // increasing left wheel speed
    }

    /*
     * This else is left here just in case if we want to let the robot operate at steady position  
     */
    /*else
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enR, 0); 
      analogWrite(enL, 0);
    }*/
    delay(T);
}

/*
float pid(int yaw, float deltat){

  int yaw_r;
  int yaw_l;
  int error1;
  int error2;
  int inv = 0;
  int normal = 0;
  
  /*
   * if the yaw angle exceeds 180 we turn the angle to be -180 and decays as the yaw increases  
   * then we check the shortest path to let the robot turn itself based on it  
   
  if(yaw > 180)
  {
    yaw_l = yaw; // saving the normal yaw angle 
    yaw_r = yaw - 360;  // saving the new orientation of the yaw angle
    error1 = setpoint - yaw_r; 
    error2 = setpoint - yaw_l;
    if (abs(error1) <= abs(error2)) // checking which error is the smallest
    {
      error = error1; 
      inv = 1; // a flag to indicate that the robot will turn in the opposote direction of the correction
    }
    else 
    {
      error = error2;
      inv = 1;
    }
  }
  else
  {
    error = setpoint - yaw; // if angle is less than 180 then the error is measured normally
  }
 
  
  
  
  integral = integral + (error) * dt;  // calculating the integral error

  /*
   * limiting the integral error
   
  if(integral > i_max)
  { 
    integral = i_max;
  }
  else if(integral < i_min)
  {
    integral = i_min;
  }

  derivative = (error - prev_error) / dt; // calculating the derivative error


  /*
   * calculating each term of the PID
   
  p_term = kp * error;

  i_term = ki * integral;

  d_term = kd * derivative;

  prev_error = error;

  pid_out = p_term + i_term + d_term;

 
 // Limiting the PID output

  if(pid_out > out_max)
  {
    pid_out = out_max;
  }
  else if(pid_out < out_min)
  {
    pid_out = out_min;
  }

  if(inv == 1) // checking the flag
  {
    pid_out = -pid_out;
  }
 


 
  Serial.print("PID : ");
  Serial.println(pid_out);
  return pid_out; 
  
}


*/
float pid(int measurment){

  int yaw_r;
  int yaw_l;
  int error1;
  int error2;
  int inv = 0;
  int normal = 0;


  if(measurment > 180)
  {
    yaw_l = measurment; // saving the normal yaw angle 
    yaw_r = measurment - 360;  // saving the new orientation of the yaw angle
    error1 = setpoint - yaw_r; 
    error2 = setpoint - yaw_l;
    if (abs(error1) <= abs(error2)) // checking which error is the smallest
    {
      error = error1; 
      inv = 1; // a flag to indicate that the robot will turn in the opposote direction of the correction
    }
    else 
    {
      error = error2;
      inv = 1;
    }
  }
  else
  {
    error = setpoint - measurment; // if angle is less than 180 then the error is measured normally
  }
 


  /*
  * Proportional
  */
  p_term = Kp * error;


  /*
  * Integral
  */
    i_term = integral + 0.5f * Ki * T * (error + prev_error);

  /* Anti-wind-up via integrator clamping */
    if ( i_term > i_max) {

         i_term = i_max;

    } else if ( i_term < i_min) {

         i_term = i_min;

    }


  /*
  * Derivative (band-limited differentiator)
  */
    
    d_term = -(2.0f * Kd * (measurment - prevMeasurment) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * tau - T) * d_term)
                        / (2.0f * tau + T);


  /*
  * Compute output and apply limits
  */
    pid_out = p_term+i_term+d_term;

    if (pid_out > out_max) {

        pid_out = out_max;

    } else if (pid_out < out_min) {

        pid_out = out_min;

    }
    if(inv == 1) // checking the flag
  {
    pid_out = -pid_out;
  }
 
  /* Store error and measurement for later use */
   prev_error = error;
   prevMeasurment = measurment;

  /* Return controller output */
    return pid_out;
  
  }
