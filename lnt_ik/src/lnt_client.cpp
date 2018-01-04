#include "ros/ros.h"
#include "lnt_ik/lnt_ik.h"
#include <lnt_packet/lnt_packet.h>


//Global variable for extracting the values from packet data
int packet;
float x,y,z,alpha,beta,gama;
float r,theta;
int mode;
//variable for storing joint angles(radians) 
double position,multiple_position[6]; 
//variable for storing the joint number
int joint_num;

void packet_data_Callback(const lnt_packet::lnt_packet::ConstPtr& packet_data)
{
   
  //Assigning the packet data to corresponding global variables
  packet = packet_data->packet_code;
  mode = packet_data->eff_mode;
  if(packet == 1)
  {
	  x = packet_data->values[0];
	  y = packet_data->values[1];
	  z = packet_data->values[2];
	  alpha = packet_data->values[3];
	  beta = packet_data->values[4];
	  gama = packet_data->values[5];
  }
  else if(packet == 2)
  {
	  r = packet_data->values[0];
	  theta = packet_data->values[1];
	  z = packet_data->values[2];
  }
  else if(packet == 5)
  {
	  joint_num = packet_data->values[0];
	  position = packet_data->values[1];
  }
  else if(packet == 6)
  {
	 for(int i=0;i<6;i++)
	 {
		 multiple_position[i] = packet_data->values[i];
	 }
 }
 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lnt_manipulator_client");
  ros::NodeHandle node_handle;
  ros::NodeHandle n;

  //creating client for lnt
  ros::ServiceClient lnt_client = n.serviceClient<lnt_ik::lnt_ik>("lnt_server");

  //Subscriber for packet data
  ros::Subscriber sub1 = node_handle.subscribe("lnt_packet_data", 1000, packet_data_Callback);

  //spinner thread for subscribers
  ros::AsyncSpinner spinner(3);
  spinner.start();
   
 //creating the srv object for request
 lnt_ik::lnt_ik srv;
  
   while(ros::ok())
  { 
	if(packet == 5)
  {
	if (lnt_client.call(srv))
  {
    ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed");
    return 1;
  }

  return 0;
}

  
  }	
  
}



