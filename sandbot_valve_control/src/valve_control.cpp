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
#include "sandbot_valve_control/ValveInput.h"
#include "sandbot_valve_control/MotorInput.h"
#include "sandbot_valve_control/OperationMode.h"
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
#define DXL_ID_10                       10                  // Dynamixel ID: 10
#define DXL_ID_13                       13                  // Dynamixel ID: 10
#define DXL_ID_254                      254
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
// #define DXL_MINIMUM_POSITION_VALUE      512            // Dynamixel will rotate between this value
// #define DXL_MAXIMUM_POSITION_VALUE      1023              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
#define VELOCITY_CONTROL_MODE           1
#define DXL_MAXIMUM_VELOCITY            380
#define CONTINUOUS_MODE                 0
#define DISCRETE_MODE                   1
#define VALVE_OPEN                      1023
int GOAL_POSITION = 512;
int MODE;
int SHUTDOWN = 0;

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

// chage valve input
void changeValveInput(int goal_position)
{
  GOAL_POSITION = goal_position;
}

// subscribe msgCallback
void msgCallback(const sandbot_valve_control::ValveInput::ConstPtr& msg1)
{
  // ROS_INFO("recieve msg1 = %d", msg1->goal_position);
  changeValveInput(msg1->goal_position);
}

void msgCallback_mode(const sandbot_valve_control::OperationMode::ConstPtr& msg_mode)
{
  MODE = msg_mode->mode;
}

void msgCallback_shutdown(const sandbot_valve_control::ShutDown::ConstPtr& msg_shutdown)
{
  SHUTDOWN = msg_shutdown->shutdown;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_control");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sandbot_valve_control::MotorInput>("motor_input", 100);

  sandbot_valve_control::MotorInput msg2;

  ros::Rate loop_rate(1);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_velocity = DXL_MAXIMUM_VELOCITY * -1;   // Goal velocity

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position
  int valve_open = VALVE_OPEN;

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

  // Change to Wheel model
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_10, ADDR_PRO_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
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
    printf("Dynamixel ID_10 is now on wheel mode \n");
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_254, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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

  // Write goal velocity
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_10, ADDR_PRO_GOAL_VELOCITY, dxl_goal_velocity, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  printf("write goal velocity done\n");

  ros::Subscriber sub = nh.subscribe("valve_input", 100, msgCallback);
  ros::Subscriber sub_mode = nh.subscribe("operation_mode", 100, msgCallback_mode);
  ros::Subscriber sub_shutdown = nh.subscribe("shut_down", 100, msgCallback_shutdown);

  // Write goal position & Shutdown case
  while(ros::ok())
  {
    // ROS_INFO("GOAL_POSITION = %d\n", GOAL_POSITION);
    ros::spinOnce();
    // Disable torque and Shutdown
    if (SHUTDOWN != 0)
    {
      // Close valve
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_GOAL_POSITION, 512, &dxl_error);
      do
      {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_13, GOAL_POSITION, dxl_present_position);
      } while((abs(GOAL_POSITION - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

      // Disable torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_254, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
    if (MODE == 0)
    {
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_GOAL_POSITION, valve_open, &dxl_error);
      // if (dxl_comm_result != COMM_SUCCESS)
      // {
      //   printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      // }
      // else if (dxl_error != 0)
      // {
      //   printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      // }
      // printf("write goal position done\n");
      do
      {
        // Read present position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        // if (dxl_comm_result != COMM_SUCCESS)
        // {
        //   printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        // }
        // else if (dxl_error != 0)
        // {
        //   printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        // }

        // printf("continuous mode\n");
        // printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_13, valve_open, dxl_present_position);

      }while((abs(valve_open - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }
    else
    {
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_GOAL_POSITION, GOAL_POSITION, &dxl_error);
      // if (dxl_comm_result != COMM_SUCCESS)
      // {
      //   printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      // }
      // else if (dxl_error != 0)
      // {
      //   printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      // }
      // printf("write goal position done\n");
      do
      {
        // Read present position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        // if (dxl_comm_result != COMM_SUCCESS)
        // {
        //   printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        // }
        // else if (dxl_error != 0)
        // {
        //   printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        // }

        // printf("discrete mode\n");
        // printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_13, GOAL_POSITION, dxl_present_position);

      }while((abs(GOAL_POSITION - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }

    // publish present position
    msg2.present_position = dxl_present_position;
    pub.publish(msg2);
    // printf("publish done\n");

    // set loop rate
    loop_rate.sleep();
  }

  // Close valve
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_GOAL_POSITION, 512, &dxl_error);
  do
  {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_13, GOAL_POSITION, dxl_present_position);
  } while((abs(GOAL_POSITION - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_254, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
