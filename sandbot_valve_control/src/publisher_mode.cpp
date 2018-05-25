#include "ros/ros.h"
#include "sandbot_valve_control/OperationMode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_input_publisher_mode");
  ros::NodeHandle nh;

  ros::Publisher pub=nh.advertise<sandbot_valve_control::OperationMode>("operation_mode", 10);
  ros::Rate loop_rate(50);

  sandbot_valve_control::OperationMode msg_mode;

  bool isContinuous= true;
  while(ros::ok()){
    if(isContinuous==true){
      msg_mode.mode=0;
    }
    else{
      msg_mode.mode=1;
    }
    pub.publish(msg_mode);
    loop_rate.sleep();
  }
  return 0;
}
