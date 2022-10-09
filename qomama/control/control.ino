#include "MPU9250.h"

MPU9250 mpu;
#include <Servo.h>

#define PID_KP  2.0f
#define PID_KI  0.1f
#define PID_KD  1.0f

#define PID_TAU 0.02f

#define PID_LIM_MIN  -90.0f
#define PID_LIM_MAX  90.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.05f

typedef struct {

  /* Controller gains */
  float Kp;
  float Ki;
  float Kd;

  /* Derivative low-pass filter time constant */
  float tau;

  /* Output limits */
  float limMin;
  float limMax;
  
  /* Integrator limits */
  float limMinInt;
  float limMaxInt;

  /* Sample time (in seconds) */
  float T;

  /* Controller "memory" */
  float integrator;
  float prevError;      /* Required for integrator */
  float differentiator;
  float prevMeasurement;    /* Required for differentiator */

  /* Controller output */
  float out;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);


Servo my_servo;
int pos;

    /* Initialise PID controller */
    PIDController pid = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
        PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };
    /* Simulate response using test system */
float setpoint=0.0;
float TestSystem_Update(float inp);
float GetSystemReading();

void setup() {
      Serial.begin(115200);

   Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
   
    my_servo.attach(9);
    PIDController_Init(&pid);

    my_servo.write(90);
    delay(1000);
    setpoint=mpu.getYaw();
}

void loop() {
    /* Get measurement from system */
    float measurement = GetSystemReading();
    Serial.println(setpoint-measurement);



    
    /* Compute new control signal */
    PIDController_Update(&pid, setpoint, measurement);
      my_servo.write(90-(PIDController_Update(&pid, setpoint, measurement)));
      delay(5);
      
      

    
}

    float GetSystemReading()
 {
  static float output;
   if (mpu.update()) {
     output = mpu.getYaw();
    // Serial.println(output);
    }

    //code to get readings from the imu sensor
    return (output);
  }

    
void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

  /*
  * Error signal
  */
    float error = setpoint - measurement;


  /*
  * Proportional
  */
    float proportional = pid->Kp * error;


  /*
  * Integral
  */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

  /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


  /*
  * Derivative (band-limited differentiator)
  */
    
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


  /*
  * Compute output and apply limits
  */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;


    }

  /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

  /* Return controller output */
    return pid->out;

}

void PIDController_Init(PIDController *pid) {

  /* Clear controller variables */
  pid->integrator = 0.0f;
  pid->prevError  = 0.0f;

  pid->differentiator  = 0.0f;
  pid->prevMeasurement = 0.0f;

  pid->out = 0.0f;

}
