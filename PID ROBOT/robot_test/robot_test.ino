/*
      This code implements a control on a two wheel differential robot the control is mainly implemented on the speed of the robot in rpm and the angle of the robot.
      The speed control is done by the optical encoder attached to each wheel of the robot. The encoder gives HIGH value each time the ir sensor catches a light
      through the encoder, this HIGH value that from the sensor is used as a tick to represent a step done by the wheel then by counting the number of ticks done by the
      wheel and measuring the diameter of the wheel then we can know the displacement the robot has made then we can calculate from it the speed in of the wheel in rpm.
      The angle of the robot is controlled by a PID controller that takes an input as a yaw angle from the imu(mpu-9255) and calcualtes the current error from the desired setpoint,
      then the PID output added or minused from the motor speeds depending on the direction of the robot.       
*/


#include "Wire.h"
// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "libs/I2Cdev.cpp"
#include "libs/MPU9250.cpp"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for Sparkfun module)
// AD0 high = 0x69
// MAHONY FILTER SELECTED BELOW

MPU9250 accelgyro;
I2Cdev   I2C_M;

//acel offsets and correction matrix
 float A_B[3] {  539.75,  218.36,  834.53}; 
 float A_Ainv[3][3]
  {{  0.51280,  0.00230,  0.00202},
  {  0.00230,  0.51348, -0.00126},
  {  0.00202, -0.00126,  0.50368}};
  
// mag offsets and correction matrix
  float M_B[3]
 {   18.15,   28.05,  -36.09};
 float M_Ainv[3][3]
 {{  0.68093,  0.00084,  0.00923},
  {  0.00084,  0.69281,  0.00103},
  {  0.00923,  0.00103,  0.64073}};
  
  float G_off[3] = {-299.7, 113.2, 202.4}; //raw offsets, determined for gyro at rest


// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

char s[60]; //snprintf buffer
//raw data and scaled as vector
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
#define gscale (250./32768.0)*(PI/180.0)  //gyro default 250 LSB per d/s -> rad/s

// NOW USING MAHONY FILTER

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define Kp 30.0
#define Ki 0.0

// globals for AHRS loop timing

unsigned long now = 0, last = 0; //micros() timers
float deltat = 0;  //loop time in seconds
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long print_ms = 1000; //print every "print_ms" milliseconds


// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output



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
float kp = 1.7;
float ki = 0.3;
float kd = 1.0;
float proportional = 0; // proportional error 
float integral = 0;     // integral error
float derivative = 0;   // derivative error
float p_term = 0; 
float i_term = 0;
float d_term = 0;
int i_max = 30;         // I term max limit
int i_min = -30;        // I term min limit
//float out_max = 50;     // output max limit
//float out_min = -50;    // output min limit
float pid_out = 0;      // PID output
long prevT = 0;        
float dt = 0;           // difference in time



// ################ SPEED CONTROL VARIABLES #################
int rpm = 150; // rpm speed  

int pwm_r = round((100 * rpm) / 345.02);  // pwm value applied on the right motor
int pwm_l = round((100 * rpm) / 310);     // pwm value applied on the left motor
float out_max = pwm_r*0.5;     // output max limit
float out_min = -pwm_r*0.5;    // output min limit




void setup() {

  
  Wire.begin();
  Serial.begin(115200);

  while(!Serial); //wait for connection

  // initialize imu device
  accelgyro.initialize();
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU9250 OK" : "MPU9250 ??");
    
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
  attachInterrupt(digitalPinToInterrupt(enc1), Inc_tics1 , RISING);
  attachInterrupt(digitalPinToInterrupt(enc2), Inc_tics2 , RISING);

  delay(3000);
  
  // This loop takes imu readings first before initializing a setpoint because first readings are not accurate
  for(int i = 0 ; i<100 ; i++)
  {
    
    setpoint = angles_calc(); // function that returnes yaw angle from the imu
    if(setpoint > 180)
    {
      setpoint -= 360; 
    }
    Serial.println(setpoint);  
  }
  
  Serial.print("Setpoint :");
  Serial.println(setpoint);

}

void loop() {
    
   
    float returned_out;  // This variable saves the output returnd from the PID  function
    long currT = micros(); // saving the current time 
    dt = (currT-prevT)/1.0e-6; // calculating the difference time 
    prevT = currT;

    g_yaw = angles_calc();  // Taking the current yaw angle from the imu 
    Serial.print("yaw : ");
    Serial.println(g_yaw);
    returned_out = pid(g_yaw, dt); 

     

       
    
    
    /* 
     * checking if the robot is moved to the right then the opposing movement to correct the error  
     * will be to the left by speeding up the right wheel and decreasing the left wheel speed
     */ 
    if(returned_out >= 0)
    {
      /*
       * The direction of the wheel is set to move forward
       */
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enR, pwm_r + abs(returned_out)); // increasing right wheel speed
      analogWrite(enL, pwm_l - abs(returned_out)); // decreasing left wheel speed   
    }

    /* 
     * checking if the robot is moved to the left then the opposing movement to correct the error  
     * will be to the right by speeding up the left wheel and decreasing the right wheel speed
     */ 
    else if(returned_out < 0)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      
      analogWrite(enR, pwm_r - abs(returned_out)); // decreasing right wheel speed
      analogWrite(enL, pwm_l + abs(returned_out)); // increasing left wheel speed
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
   
}


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
   */
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
   */
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
   */
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




int angles_calc()
{
  get_MPU_scaled();
  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  last = now;

  // correct for differing accelerometer and magnetometer alignment by circularly permuting mag axes

MahonyQuaternionUpdate(-Axyz[0], Axyz[1], Axyz[2], Gxyz[0], -Gxyz[1], -Gxyz[2],
                         Mxyz[1], -Mxyz[0], Mxyz[2], deltat);
  //  Standard orientation: X North, Y West, Z Up
  //  Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
 
  // Strictly valid only for approximately level movement       
  // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
  // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
  roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
  // to degrees
  yaw   *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  
  // http://www.ngdc.noaa.gov/geomag-web/#declination
  //conventional nav, yaw increases CW from North, corrected for local magnetic declination

  yaw = -yaw + 14.5;
  if(yaw<0) yaw += 360.0;
  if(yaw>360.0) yaw -= 360.0;
  now_ms = millis(); //time to print?
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;
    // print angles for serial plotter...
    //  Serial.print("ypr ");
    //Serial.print(yaw, 0);
    //Serial.print(", ");
    //Serial.print(pitch, 0);
    //Serial.print(", ");
    //Serial.println(roll, 0);
  }
  return yaw;
}





//Function of interrupt to increase the tics number of encoder of motor B
void Inc_tics1(){
  N_tics1_curr+=1;
  }

//Function of interrupt to increase the tics number of encoder of motor A
void Inc_tics2(){
  N_tics2_curr+=1;
  }
  
//Function to limit the tics number of encoders to avoid exceeding the storage limit
//void limit_counter(){
  //N_tics1-=1000;
  //N_tics2-=1000;
  //}/

//Function to reset the tics number of encoders if the pilot stops the robot 
//void reset_counter(){
  //N_tics1=N_tics2=0;
  //}




void get_MPU_scaled(void) {
  float temp[3];
  int i;
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
  Gxyz[1] = ((float) gy - G_off[1]) * gscale;
  Gxyz[2] = ((float) gz - G_off[2]) * gscale;

  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;
  //apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  Mxyz[0] = (float) mx;
  Mxyz[1] = (float) my;
  Mxyz[2] = (float) mz;
  //apply offsets and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
 }

// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;
  
    // already done in loop()

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;
  
  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of the reference vectors
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki*eInt[0];
    gy += Ki*eInt[1];
    gz += Ki*eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex; 
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;

// Integrate rate of change of quaternion
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}  
