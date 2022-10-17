#include <math.h>

float PID_controller(float x, float y, float theta, float setpoint_x, float setpoint_y)
{
//PID constants initialization
float kp=1;
float ki=0;
float kd=0;

//error calculation
float error_x=setpoint_x-x;
float error_y=setpoint_y-y;
if(abs(error_x)<0.05 && abs(error_y)<0.05 )
{
 return 0;
}
if(error_x==0)
{

}
float error_theta=atan2(error_y,error_x);
error_theta=error_theta*(180/M_PI);
float P_value_x =

}
