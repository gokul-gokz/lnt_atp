#include "ros/ros.h"
#include "lnt_ik/lnt_ik.h"
#include <lnt_packet/lnt_packet.h>
#include <stdio.h>




class lnt_data
{
 private:
	//variable for storing safety limits for each joints
	double max[6] = {3.14,3.14,0,2.09,1.57,3.14};
double min[6] = {-3.14,0,-3.14,-2.09,-1.57,-3.14};

 public:
	//Global variable for extracting the values from packet data
	int packet=0;
	float x,y,z,alpha,beta,gama;
	float r,theta;
        float values[6]; 
         
	int mode;
	ros::NodeHandle n;
	//variable for storing joint angles(radians) 
	double position=50;
	double multiple_position[6]; 
	//variable for storing the joint number
	int joint_num=0;

	void packet_data_Callback(const lnt_packet::lnt_packet::ConstPtr& packet_data);

        bool safety_check(int joint, double position);

        bool calling_service();
};


//Safety limit check function
bool lnt_data::safety_check(int joint, double position)
{
  if ((position>=min[joint]) && (position<=max[joint]))
    return true;
  else
    return false;

}

void lnt_data::packet_data_Callback(const lnt_packet::lnt_packet::ConstPtr& packet_data)
{
   
  //Assigning the packet data to corresponding global variables
  ROS_INFO("packet=%d",packet);
  packet = packet_data->packet_code;
  mode = packet_data->eff_mode;
   for(int i=0;i<6;i++)
	 {
		 values[i] = packet_data->values[i];
	 }
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
	  //Radians Conversion
	  position = (position *3.14)/180;
	  std::cout<<"position"<<position;
	  
          

  }
  else if(packet == 6)
  {
	 for(int i=0;i<6;i++)
	 {
		 multiple_position[i] = packet_data->values[i];
	 }
 }
 
}

bool lnt_data::calling_service()
{
	 
	 //ROS_INFO("packet=%d",packet);
     ros::NodeHandle n;
     ROS_INFO("calling service");
     
	 if(lnt_data::safety_check(joint_num,position))
	  {
	   //creating client for the server joint_space_control_individual
       ros::ServiceClient lnt_client = n.serviceClient<lnt_ik::lnt_ik>("joint_space_control_individual");
	   
	   lnt_ik::lnt_ik srv;
	   for(int i=0;i<6;i++)
	 {
		 srv.request.values[i] = values[i];
	 }
		   
           lnt_client.call(srv);
           ros::Duration(5).sleep();
           
   }
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lnt_manipulator_client");
  ros::NodeHandle node_handle;


  

 
  //Subscriber for packet data
  lnt_data l1;
  ros::Subscriber sub1 = node_handle.subscribe("lnt_packet_data", 1000, &lnt_data::packet_data_Callback, &l1);

  
  //ROS_INFO("calling service");
  l1.calling_service();
 
  //spinner thread for subscribers
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  
   
  return 0;

 	
  }



