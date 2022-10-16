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
std_msgs::String speeds;
ros::Publisher chatter("chatter", &speeds);

void messageCb( const std_msgs::String& msg){

char mymessage[9];
char updated_msg[7];
for (char i =1 ;i<=8;i++)
{
  mymessage[i-1] = msg.data[i];
  
}
if ((sizeof(mymessage)/sizeof(mymessage[0]))==1)
{
  if ((strcmp(mymessage[0],"#"))&&(strcmp(mymessage[8],"#")))
  {
    for (char i=0;i<7;i++)
    {
      updated_msg[i] = mymessage[i+1]; 
      
    }
    
//    mymessage.remove(0);
//    mymessage.remove(8);
    speeds.data = updated_msg;    
    
  }
  
}

  nh.loginfo ( speeds.data );
}

ros::Subscriber<std_msgs::String> sub("/toROV", messageCb );


void setup() {
  // put your setup code here, to run once:
    nh.initNode();
    nh.subscribe(sub);
      nh.advertise(chatter);
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

    
    //delay(500);
}

int motion(int speedsArr)
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

}
