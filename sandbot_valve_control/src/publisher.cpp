#include "ros/ros.h"
#include "sandbot_valve_control/ValveInput.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_input_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub=nh.advertise<sandbot_valve_control::ValveInput>("valve_input", 10);
  ros::Rate loop_rate(50);

  sandbot_valve_control::ValveInput msg;

  bool isOpen= true;
  //Example of open and close alternatively
  while(ros::ok()){
    if(isOpen==true){
      msg.goal_position=2048;  //command to close
      isOpen=false;
    }
    else{
      msg.goal_position=2550; //command to open
      isOpen=true;
    }
    pub.publish(msg);
    loop_rate.sleep();
  }
  return 0;
}
