
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
char speeds[10]="";
boolean check=false;
char hash = '#';

void messageCb(const std_msgs::String& msg){
  speeds[0]=msg.data[0];
  speeds[8]=msg.data[8];
//msg.data.toCharArray(speeds,10);
//speeds=msg.data;

check_msg();
/*if(check)
{
  nh.loginfo("True");
}
else
{
  nh.loginfo("False");
}*/
//nh.loginfo(speeds[8]);
nh.loginfo(msg.data);


  
}

ros::Subscriber<std_msgs::String> sub("/toROV", messageCb );

void check_msg()
{
  boolean x=(speeds[0]==hash);
  boolean y=(speeds[8]==hash);
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


}

void loop() {
  // put your main code here, to run repeatedly:
    nh.spinOnce();
    //delay(500);
}
