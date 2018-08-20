#include "ros/ros.h"
#include "sandbot_valve_control/IntensityInput.h"
#include "sandbot_valve_control/HeightInput.h"
// #include "sandbot_valve_control/DistanceInput.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_input_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub_intensity = nh.advertise<sandbot_valve_control::IntensityInput>("intensity", 10);
  ros::Publisher pub_height = nh.advertise<sandbot_valve_control::HeightInput>("height", 10);
  // ros::Publisher pub_distance = nh.advertise<sandbot_valve_control::DistanceInput>("distance", 10);
  ros::Rate loop_rate(50);

  sandbot_valve_control::IntensityInput msg_intensity;
  sandbot_valve_control::HeightInput msg_height;
  // sandbot_valve_control::DistanceInput msg_distance;

  bool ChangeDirection = true;
  int intensity_input = 1000;
  int height_input = 3095;
  // int distance_input = 814;
  int delta = 5;
  //Example of moving direction alternatively
  while(ros::ok()){
    if(ChangeDirection == true){
      if(intensity_input > 680+delta){
        intensity_input = intensity_input-delta;
      }
      if(height_input > 480+delta){
        height_input = height_input-delta;
      }
      // if(distance_input < 3910-delta){
      //   distance_input = distance_input+delta;
      // }
      msg_intensity.intensity_input = intensity_input;
      msg_height.height_input = height_input;
      // msg_distance.distance_input = distance_input;
      // if(intensity_input <= 680+delta && height_input <= 480+delta && distance_input >= 3910-delta){
      if(intensity_input <= 680+delta && height_input <= 480+delta){
        ChangeDirection = false;
      }
      // msg.goal_position=2020;  //command to close
    }
    else{
      if(intensity_input < 1000-delta){
        intensity_input = intensity_input+delta;
      }
      if(height_input < 3095-delta){
        height_input = height_input+delta;
      }
      // if(distance_input > 814+delta){
      //   distance_input = distance_input-delta;
      // }
      msg_intensity.intensity_input = intensity_input;
      msg_height.height_input = height_input;
      // msg_distance.distance_input = distance_input;
      // if(intensity_input >= 1000-delta && height_input >= 3095-delta && distance_input <= 814+delta){
      if(intensity_input >= 1000-delta && height_input >= 3095-delta){
        ChangeDirection = true;
      }
    }
    pub_intensity.publish(msg_intensity);
    pub_height.publish(msg_height);
    // pub_distance.publish(msg_distance);
    loop_rate.sleep();
  }
  return 0;
}
