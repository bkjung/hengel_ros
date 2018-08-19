#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

// #include "sandbot_valve_control/DynamixelSDK.h"                                  // Uses Dynamixel SDK library
#include "ros/ros.h"
// #include "sandbot_valve_control/ValveInput.h"
// #include "sandbot_valve_control/MotorInput.h"
#include "sandbot_valve_control/IntensityInput.h"
#include "sandbot_valve_control/HeightInput.h"
#include "sandbot_valve_control/DistanceInput.h"
// #include "sandbot_valve_control/OperationMode.h"
#include "sandbot_valve_control/ShutDown.h"
#include "sandbot_valve_control/packet_handler.h"
#include "sandbot_valve_control/port_handler.h"

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_VELOCITY          104
#define ADDR_PRO_PRESENT_VELOCITY       128
#define ADDR_PRO_OPERATING_MODE         11


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_INTENSITY                   10                  // Dynamixel ID: 10
#define DXL_DISTANCE                    20                  // Dynamixel ID: 20
#define DXL_HEIGHT                      30                  // Dynamixel ID: 30
#define DXL_ALL                         254
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
// #define DXL_MINIMUM_POSITION_VALUE      512            // Dynamixel will rotate between this value
// #define DXL_MAXIMUM_POSITION_VALUE      1023              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
// #define VELOCITY_CONTROL_MODE           1
// #define DXL_MAXIMUM_VELOCITY            380

// int GOAL_POSITION = 1024;

// motor input
int INTENSITY = 960;
int DISTANCE = 814;
int HEIGHT = 3012;

int SHUTDOWN = 0;
// bool RECEIVED_MSG = false;

#define ESC_ASCII_VALUE                 0x1b

// get char
int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

// change intensity input
void changeIntensityInput(int goal_position)
{
  INTENSITY = goal_position;
}
// change height
void changeHeightInput(int goal_position)
{
  HEIGHT = goal_position;
}
// change distance
void changeDistanceInput(int goal_position)
{
  DISTANCE = goal_position;
}

// subscribe msgCallback
void msgCallbackIntensity(const sandbot_valve_control::IntensityInput::ConstPtr& msg_intensity)
{
  ROS_INFO("recieved msg_intensity = %d", msg_intensity->intensity_input);
  changeIntensityInput(msg_intensity->intensity_input);
  // RECEIVED_MSG = true;
}
void msgCallbackHeight(const sandbot_valve_control::HeightInput::ConstPtr& msg_height)
{
  ROS_INFO("recieved msg_height = %d", msg_height->height_input);
  changeHeightInput(msg_height->height_input);
  // RECEIVED_MSG = true;
}
void msgCallbackDistance(const sandbot_valve_control::DistanceInput::ConstPtr& msg_distance)
{
  ROS_INFO("recieved msg_distance = %d", msg_distance->distance_input);
  changeDistanceInput(msg_distance->distance_input);
  // RECEIVED_MSG = true;
}
void msgCallbackShutdown(const sandbot_valve_control::ShutDown::ConstPtr& msg_shutdown)
{
  SHUTDOWN = msg_shutdown->shutdown;
}
// void msgCallback(const sandbot_valve_control::ValveInput::ConstPtr& msg1)
// {
//   ROS_INFO("recieved msg1 = %d", msg1->goal_position);
//   changeValveInput(msg1->goal_position);
//   RECEIVED_MSG = true;
// }
//
// void msgCallback_mode(const sandbot_valve_control::OperationMode::ConstPtr& msg_mode)
// {
//   ROS_INFO("recieved msg_mode = %d", msg_mode->mode);
//   MODE = msg_mode->mode;
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_control");
  ros::NodeHandle nh;
  // ros::Publisher pub = nh.advertise<sandbot_valve_control::MotorInput>("motor_input", 100);

  // sandbot_valve_control::MotorInput msg2;

  ros::Rate loop_rate(50);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ALL, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  ros::Subscriber sub_intensity = nh.subscribe("intensity", 100, msgCallbackIntensity);
  ros::Subscriber sub_height = nh.subscribe("height", 100, msgCallbackHeight);
  ros::Subscriber sub_distance = nh.subscribe("distance", 100, msgCallbackDistance);
  ros::Subscriber sub_shutdown = nh.subscribe("shut_down", 100, msgCallbackShutdown);
  // ros::Subscriber sub = nh.subscribe("valve_input", 100, msgCallback);

  // Write goal position & Shutdown case
  while(ros::ok())
  {
    ros::spinOnce();
    // Disable torque and Shutdown
    if (SHUTDOWN != 0)
    {
      // Goal position example
      // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_GOAL_POSITION, 960, &dxl_error);
      // if (dxl_comm_result != COMM_SUCCESS)
      // {
      //   printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      // }
      // else if (dxl_error != 0)
      // {
      //   printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      // }
      // printf("write goal position done\n");
      // dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      // printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_13, GOAL_POSITION, dxl_present_position);
      // do
      // {
      //   dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      //   printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_13, GOAL_POSITION, dxl_present_position);
      //
      //   //if (RECEIVED_MSG == true)
      //   //{
      //   //    RECEIVED_MSG = false;
      //   //    break;
      //   //}
      // } while((abs(GOAL_POSITION - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD) && ros::ok());

      // Default Position
      // Intensity
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_INTENSITY, ADDR_PRO_GOAL_POSITION, 960, &dxl_error);
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_INTENSITY, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_INTENSITY, 960, dxl_present_position);
      // Height
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_HEIGHT, ADDR_PRO_GOAL_POSITION, 3012, &dxl_error);
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_HEIGHT, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_HEIGHT, 3012, dxl_present_position);
      // Distance
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_DISTANCE, ADDR_PRO_GOAL_POSITION, 814, &dxl_error);
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_DISTANCE, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_DISTANCE, 814, dxl_present_position);
      printf("dynamixel is now on default position");

      // Disable torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ALL, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }
      printf("dynimixel torque disabled\n");

      // Shutdown
      ros::shutdown();
    }

    // Write goal position
    // Intensity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_INTENSITY, ADDR_PRO_GOAL_POSITION, INTENSITY, &dxl_error);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_INTENSITY, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_INTENSITY, INTENSITY, dxl_present_position);
    // Height
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_HEIGHT, ADDR_PRO_GOAL_POSITION, HEIGHT, &dxl_error);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_HEIGHT, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_HEIGHT, HEIGHT, dxl_present_position);
    // Distance
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_DISTANCE, ADDR_PRO_GOAL_POSITION, DISTANCE, &dxl_error);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_DISTANCE, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_DISTANCE, DISTANCE, dxl_present_position);

    // // publish present position
    // msg2.present_position = dxl_present_position;
    // pub.publish(msg2);
    // // printf("publish done\n");

    // set loop rate
    loop_rate.sleep();
  }

  // Default Position
  // Intensity
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_INTENSITY, ADDR_PRO_GOAL_POSITION, 960, &dxl_error);
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_INTENSITY, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
  printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_INTENSITY, 960, dxl_present_position);
  // Height
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_HEIGHT, ADDR_PRO_GOAL_POSITION, 3012, &dxl_error);
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_HEIGHT, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
  printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_HEIGHT, 3012, dxl_present_position);
  // Distance
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_DISTANCE, ADDR_PRO_GOAL_POSITION, 814, &dxl_error);
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_DISTANCE, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
  printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_DISTANCE, 814, dxl_present_position);
  printf("dynamixel is now on default position");

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ALL, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  printf("dynimixel torque disabled\n");

  // Close port
  portHandler->closePort();
  printf("port closed\n");

  return 0;
}
