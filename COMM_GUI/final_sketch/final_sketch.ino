#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>


/*############# DEFINING MOTORS PINS #############*/
#define FR_pin 2
#define FL_pin 3
#define BR_pin 4
#define BL_pin 5
#define TF_pin 6
#define TB_pin 7


Servo FR;
Servo FL;
Servo BL;
Servo BR;
Servo TF;
Servo TB;

/*initialising ROV node and publisher*/
ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher ROV("/fromROV", &str_msg);

/*initialising global variables and character lists*/
char speeds[10]="";
char speedsArr[7]={};
//char bufferr[10]="";
unsigned int buttons_bin=0;
boolean buttonArr[6]={0,0,0,0,0};
boolean check=false;

void messageCb(const std_msgs::String& msg) /*callback function for subscriber*/
{
  
  strcpy(speeds,"000000000");               /*resetting speeds list*/
  int n= strlen((const char*)msg.data);     /*detecting the number of characters in the recieved msg*/

  for (int i=0; i<n; i++)                   /*copying values from msg to speed*/
  {
    speeds[i]=msg.data[i];
  }

  check_msg((String)msg.data);                              /*calling fn to check on the end and beginning of recieved msg*/

  if(check)                                 /*proceeds if the msg is correctly recieved*/
  {
    nh.loginfo("True");                     /*decoding values stored in msg into decimal values*/
      
    for (int i=0;i<=6;i++)
    {
      speedsArr[i]=((int)speeds[i+1]);
    }
    buttons_bin=((int)speeds[7]-50,BIN);       /*converting buttons character into the binary equivalent number*/
    int temp=0;
    for(int i=0;i<=5;i++)                   /*passing the value of each button state into nthe buttons array*/
    {
      temp=buttons_bin/10;
      buttonArr[i]=temp;
      buttons_bin=buttons_bin-temp*(10^i);
    }
    motion(speedsArr);        
    str_msg.data=speeds;
    ROV.publish(&str_msg);
  }
  else
  {
    nh.loginfo("False");
  }
    
  nh.loginfo(msg.data);
}

ros::Subscriber<std_msgs::String> sub("/toROV", messageCb );


void check_msg(String speedss)
{
  boolean x=speedss.startsWith("#");
  boolean y=speedss.endsWith("#");
  if ((x)&&(y))
  {
    check=true;
  }
  
  else
  {
    check=false;
  }
}

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(ROV);
    
    pinMode(FR_pin, OUTPUT);
    pinMode(FL_pin, OUTPUT);
    pinMode(BR_pin, OUTPUT);
    pinMode(BL_pin, OUTPUT);
    pinMode(TF_pin, OUTPUT);
    pinMode(TB_pin, OUTPUT);


    /* INITIALIZING THRUSTERS 
       Zero position of thrusters is at 1500.
       if thruster takes more than 1500 it rotates clock wise
       if thruster takes less than 1500 it rotates anticlock wise
    */ 
    FR.writeMicroseconds(1500);   //Front right thruster
    FL.writeMicroseconds(1500);   //Front left thruster
    BR.writeMicroseconds(1500);   //Back right thruster
    BL.writeMicroseconds(1500);   //Back left thruster
    TF.writeMicroseconds(1500);   //Top front thruster
    TB.writeMicroseconds(1500);   //Top back thruster
    delay(7000);

}

void loop() {
    nh.spinOnce();
    //delay(500);
}


/*int motion(int speedsArr[6])
=======
/*
*this function takes the speeds given to thrusters from motion.py and add each speed to its specific thruster so that the speed of each thruster change as the axis of the joystick changes.
*first step is to assign the values of motor speeds tp tspeed array 
*second step is to make sure that values are not out of limit (-200,200)
*if the values are out of range, calculate the scaling factor and multiply the speeds by this factor to get into the range and avoid hardware problems
*if values are in range skip the loop of scaling
*third step is to add the speeds of motors to the setpoint so it can be varied from 1300:1700
*/

int motion(char speedsArr[7])
{
  //local variable declerations
  int tspeed[6];
  int max_speed=200, min_speed=-200;    //hardware limitition for the thruster speeds
  float scaling;

  //the loop that check if values are in limited range or out of range
  //at the end of the loop scaling factor is calculated
  for (int i = 0 ; i<6 ; i++)
  {
    tspeed[i] = speedsArr[i];
    if (tspeed[i]>max_speed){
      max_speed=tspeed[i];
      }
    else if (tspeed[i]<min_speed){
      min_speed=tspeed[i];
      }
    if (-min_speed>max_speed){
      scaling=200/-min_speed;
      }
    else if (-min_speed<max_speed){
      scaling=200/max_speed;
      }
    else{scaling=200/max_speed;}
  }

  //restate the max and min speeds to 200 and -200 for limit range boundaries
  max_speed=200;
  min_speed=-200;

  //scaling the speeds if they are out of range
if (scaling!=1)
{

for (int i = 0 ; i<6 ; i++)
{
tspeed[i]=tspeed[i]*scaling; 

}



//assign the speeds to the thrusters
}
  FR.writeMicroseconds(1500 + tspeed[0]);
  FL.writeMicroseconds(1500 + tspeed[1]);
  TF.writeMicroseconds(1500 + tspeed[2]);
  TB.writeMicroseconds(1500 + tspeed[3]);
  BR.writeMicroseconds(1500 + tspeed[4]);
  BL.writeMicroseconds(1500 + tspeed[5]);

}
