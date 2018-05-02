#include "ros/ros.h"
#include "sandbot_valve_control/ValveInput.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_input_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub=nh.advertise<sandbot_valve_control::ValveInput>("valve_input", 10);
  ros::Rate loop_rate(1);

  sandbot_valve_control::ValveInput msg;

  bool isOpen= true;
  while(ros::ok()){
    if(isOpen==true){
      msg.goal_position=512;
      isOpen=false;
    }
    else{
      msg.goal_position=1023;
      isOpen=true;
    }
    pub.publish(msg);
    loop_rate.sleep();
  }
  return 0;
}
