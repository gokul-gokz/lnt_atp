/*******************************************************************************
* This header files defines the class that can used to interact with single dynamixel motor.
* This builds on top of dynamixel sdk package and hence make sure they are available before 
compiling. Make appropriate changes to CMakeLists.txt if necessary.
* This class was built based upon example/protocol1.0/read_write.cpp.
*******************************************************************************/

#ifndef DXL_DRIVER
#define DXL_DRIVER

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"    
#include "ros/ros.h"
#include <string>
#include <map>
#include <vector>
#include <iostream>

#include <inttypes.h>

#include <time.h>
 
// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_LED_RED                563
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

#define ADDR_MX_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION          116
#define ADDR_MX_PRESENT_POSITION       132

// Data Byte Length
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4


#define LEN_MX_GOAL_POSITION           4
#define LEN_MX_PRESENT_POSITION        4


// Protocol version
#define PROTOCOL_VERSION                2.0    

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
using namespace std;


class dxl_driver{
private:

int baudrate_,num_of_joints_;
std::string devicename_;
std::map<std::string,int> joint_ids_,joint_nums_,joint_speed_,joint_pos_min_,joint_pos_max_,joint_pos_init_;


dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
dynamixel::GroupBulkWrite *groupBulkWrite;
dynamixel::GroupBulkRead *groupBulkRead;



int dxl_comm_result;		
uint8_t dxl_error;                     // Dynamixel error
int dxl_goal_position;          // goal position value
uint8_t param_goal_position[4];		//array for low and high bytes
bool dxl_addparam_result;
bool dxl_getdata_result;
int index;
ros::NodeHandle nh_;

public:

dxl_driver(ros::NodeHandle &nh); 		//constructor
~dxl_driver(); 		//destructor

void init();

 
void sleep(unsigned int mseconds);

int getch();
int kbhit(void);
void printmap(std::map<std::string,int> &m);//for printing maps read from manipulator.yaml 
void printvector(std::vector<double> &vec);
void printvariables();

void read(std::vector<double> &joint_position,std::vector<double> &joint_velocity,std::vector<double> &joint_effort);
void write(std::vector<double> &joint_position_command,std::vector<double> &joint_velocity_command,std::vector<double> &joint_effort_command);

void openPort();
void setBaudRate();
void enableTorque();
void add_read_param(); 
void disableTorque(); 
void closePort();


};

#endif 
