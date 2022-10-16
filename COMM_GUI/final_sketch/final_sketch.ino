
#include <ros.h>
#include <std_msgs/String.h>

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


}

void loop() {
  // put your main code here, to run repeatedly:
    nh.spinOnce();
    //delay(500);
}
