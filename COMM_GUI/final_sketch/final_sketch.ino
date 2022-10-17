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


ros::NodeHandle  nh;
char speeds[10]="";
char ActualSpeeds[6]="";
char buttons[]="";
boolean check=false;
char hash = '#';

void messageCb(const std_msgs::String& msg){
strcpy(speeds,"000000000");
  //speeds[0]=msg.data[0]; 
  //speeds[8]=msg.data[8];
int n= strlen((const char*)msg.data);

for (int i=0; i<n; i++)
{
  speeds[i]=msg.data[i];
}
check_msg();
if(check)
{
  nh.loginfo("True");
}
else
{
  nh.loginfo("False");
}
//nh.loginfo(speeds[8]);
nh.loginfo(msg.data);


  
}

ros::Subscriber<std_msgs::String> sub("/toROV", messageCb );

void check_msg()
{
  boolean x=(speeds[0]==hash);
  boolean y=(speeds[strlen(speeds)-1]==hash);
  if ((x )&&(y))
  {
    check=true;
  }
  else
  {
    check=false;
  }
}

void setup() {
  // put your setup code here, to run once:
    nh.initNode();
    nh.subscribe(sub);
    
    
    pinMode(FR_pin, OUTPUT);
    pinMode(FL_pin, OUTPUT);
    pinMode(BR_pin, OUTPUT);
    pinMode(BL_pin, OUTPUT);
    pinMode(TF_pin, OUTPUT);
    pinMode(TB_pin, OUTPUT);


    /* INITIALIZING THRUSTERS */ 
    FR.writeMicroseconds(1500);
    FL.writeMicroseconds(1500);
    BR.writeMicroseconds(1500);
    BL.writeMicroseconds(1500);
    TF.writeMicroseconds(1500);
    TB.writeMicroseconds(1500);
    delay(7000);

}

void loop() {
  // put your main code here, to run repeatedly:
    nh.spinOnce();
    //motion(speedsArr);
    
    //delay(500);
}

/*int motion(int speedsArr[6])
{
  int tspeed[6];
  for (int i = 0 ; i<6 ; i++)
  {
    tspeed[i] = speedsArr[i];
  }

  FR.writeMicroseconds(1500 + tspeed[0]);
  FL.writeMicroseconds(1500 + tspeed[1]);
  TF.writeMicroseconds(1500 + tspeed[2]);
  TB.writeMicroseconds(1500 + tspeed[3]);
  BR.writeMicroseconds(1500 + tspeed[4]);
  BL.writeMicroseconds(1500 + tspeed[5]);

}*/
