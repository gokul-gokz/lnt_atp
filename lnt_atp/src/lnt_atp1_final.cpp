/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Gokul */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//Visulaization tools
#include <moveit_visual_tools/moveit_visual_tools.h>

//Standard header files
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

//For mathematical calculation and conversion
#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_datatypes.h>

//custom message

#include <lnt_packet/lnt_packet.h>

//variable for getting joint_values of robot
float joint_val[6];

//Global variable for extracting the values from packet data
int packet;
float x,y,z,alpha,beta,gama;
float r,theta;
int mode;
//variable for storing joint angles(radians) 
double position,multiple_position[6]; 
//variable for storing the joint number
int joint_num;

//Check variable for subscriber callback 
bool subscriber_check =false;

//variable to check the theta state in cylidrical goal and other states
bool flag=1;


//variable for storing safety limits for each joints
double max[6] = {3.14,3.14,0,2.09,1.57,3.14};
double min[6] = {-3.14,0,-3.14,-2.09,-1.57,-3.14};

//Safety limit check function
bool safety_check(int joint, double position)
{
  if ((position>=min[joint]) && (position<=max[joint]))
    return true;
  else
    return false;
}

//Subscriber callback function for publishing values
void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& jointstates)
{
  for(int i=1;i<=6;i++)
  {
   
   if(i<=6)
   {
	  joint_val[i] = jointstates->position[i];
   }
  }
  }

//Subscriber callback function for getting packet data
void packet_data_Callback(const lnt_packet::lnt_packet::ConstPtr& packet_data)
{
   subscriber_check = true;
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
  ros::init(argc, argv, "lnt_position_control");
  ros::NodeHandle node_handle;
  ros::NodeHandle n;
  
  //Subscriber for joint positions
  ros::Subscriber sub = node_handle.subscribe("joint_states", 1000, joint_states_Callback);
  
  //Subscriber for packet data
  ros::Subscriber sub1 = node_handle.subscribe("lnt_packet_data", 1000, packet_data_Callback);
  
    
  
   
  //spinner thread for subscribers
  ros::AsyncSpinner spinner(3);
  spinner.start();
  
  //Define the planning group 
  static const std::string PLANNING_GROUP = "manipulator";

  //Invoke the movegroup interface for the planning group which we want
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  //Test code
  
  
	
  
  // Visualization
  // ^^^^^^^^^^^^^
  // Adding visualization tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  
  
  // Rviz provides many types of markers, in this test we will use text
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.75; // above head of PR2
  visual_tools.publishText(text_pose, "L&T Manipulator", rvt::WHITE, rvt::XLARGE);
  
  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();
    
  bool success = true;
  bool continuous_command = true;
  while(ros::ok())
  { 
  if(subscriber_check && continuous_command && success )
  {  
  //Packet data:5 - Publishing individual joint commands
  if(packet == 5)
  {
	     
	  //Radians Conversion
	  position = (position *3.14)/180;
	  
	  //Check safety limits and execute
	 if(safety_check(joint_num,position))  
     {
      //Create a robotstate object and store the current state information(position/accleration/velocity)
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // Modify the joint state accordingly
      joint_group_positions[joint_num] = position;  // radians
      move_group.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      success = move_group.plan(my_plan);
      ROS_INFO_NAMED("Executing joint space goal %s", success ? "" : "FAILED");

      // Visualize the plan in Rviz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
	  
	  //Execute in real hardware
	  move_group.move(); 
	  //Mark the end of execution
      subscriber_check = false;
      continuous_command =true;
	 }
	 else
	 {
		 std::cout<<"Value:Out of range";	 
    }
   }
   //Packet data:6 - Publishing multiple joint commands at same time
   else if(packet==6)
   {
	 	 
	 for(int i=0;i<6;i++)
	 {
	  multiple_position[i] = (multiple_position[i] *3.14)/180;
     } 
	 
		 
	  //Create a robotstate object and store the current state information(position/accleration/velocity)
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // Modify the joint state accordingly
      for(int i=0;i<6;i++)
      {
	   if(safety_check(i,multiple_position[i]))
	   {	  
        joint_group_positions[i] = multiple_position[i];
	   }
	   else
	   {
		   std::cout<<"Joint"<<i<<" out of range"<<std::endl;
       }
      }
      move_group.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      success = move_group.plan(my_plan);
      ROS_INFO_NAMED("Executing joint space goal(Multiple Joints) %s", success ? "" : "FAILED");

      // Visualize the plan in Rviz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Joint Space Goal(Multiple)", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
	  
	  //Execute in real hardware
	  move_group.move();
	  
	  //Mark the end of execution
      subscriber_check = false; 
      continuous_command = true; 
	  }
   
  else if(packet == 1 || packet == 2)
  { 
      //Global variable 'packet' suddenly becomes zero inbetween,reason need to be findout,so creating a temporary variable
	  int packet1=packet;	  
	  // Case:1 Cylindrical control mode -only changing theta
	 if(packet == 2 && theta != 0 && r == 0 && z==0)
	 {
		 
	  //Create a robotstate object and store the current state information(position/accleration/velocity)
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
      
      //Finding the current position(angle) of base joint and incrementing it with theta provided
	  //Radians Conversion
	  position = (theta *3.14)/180 +  joint_group_positions[0];
	  
	  std::cout<<"position"<<position;
	  
	  //Checking the limits
	  if(position>3.14)
	  {
		  position = -(6.28-position);
	  }
	  else if(position<-3.14)
	  {
		  position = 6.28 + position;
	  }
	 std::cout<<"Entering cylindrical mode-theta change";
	 //Check safety limits and execute
	 if(safety_check(0,position))  
     {
     
      // Modify the joint state accordingly
      joint_group_positions[0] = position;  // radians
      move_group.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      success = move_group.plan(my_plan);
      ROS_INFO_NAMED("Executing cylindrical space goal %s", success ? "" : "FAILED");

      // Visualize the plan in Rviz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Cylindrical space Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
	  
	  //Execute in real hardware
	  move_group.move(); 
	  //Mark the end of execution
      subscriber_check = false;
      continuous_command =true;
      flag =0;
      
	 }
	 else
	 {
		 std::cout<<"Value:Out of range";	
		 flag = 0;
		 
    }
   }
   
    //Case 2- cylindrical mode ,changing multiple values
	if(packet == 2 && (((theta == 0 && r != 0 || z!=0))||flag==1))
	{
		
	  
	  //For calculation of initial extension of the arm 'R'
		  float x1,y1,R;
   		  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
   		  x1 = current_pose.pose.position.x;
   		  y1 = current_pose.pose.position.y;
   		  R = sqrt(pow(x1,2)+pow(y1,2));
   		  //Adding the initial extension of the arm with required additional movement
   		  R = R+r;
   		  std::cout<<"x1="<<x1;
   		  std::cout<<"y1="<<y1;
   		  std::cout<<"R="<<R;
   		  
     //For calculation of initial theta of end effector
      float theta1;
      theta1 = atan(y1/x1);
      std::cout<<"theta1"<<theta1;
   		  
	  position = (theta *3.14)/180 +  theta1;
	  
	  std::cout<<"position"<<position;
	  
	  //Checking the limits
	  if(position>3.14)
	  {
		  position = -(6.28-position);
	  }
	  else if(position<-3.14)
	  {
		  position = 6.28 + position;
	  }
		 
		 
		 
   		  
      	  //Cylindrical to cartesian conversion
		  x= R*cos(position);
		  std::cout<<"x="<<x;
		  y= R*sin(position);
		  std::cout<<"y="<<y;
	      mode = 1;
	      flag == 1;
	   }
	   
	 std::cout<<"packet1"<<packet1;
	 
	 if (packet1 ==1)
	 {
		 flag=1;
	 }
	 
	if(flag = 1)
	{	
	// Assigning the current position as start position
    geometry_msgs::Pose start_pose;
    
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
         
    start_pose.position.x = current_pose.pose.position.x;
    start_pose.position.y = current_pose.pose.position.y;
    start_pose.position.z = current_pose.pose.position.z; 
    start_pose.orientation.x =  current_pose.pose.orientation.x;
    start_pose.orientation.y = current_pose.pose.orientation.y;
    start_pose.orientation.z = current_pose.pose.orientation.z;
    start_pose.orientation.w = current_pose.pose.orientation.w;
       
    
    //Create a plan object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
   
    //End Effector Mode selection
   	switch(mode)
   	{
		case 1:
		{
	   //Planning with orientation Constraints
        moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "lnt_gripper_tool_frame";
		ocm.header.frame_id = "base_link";
		ocm.orientation.x =  start_pose.orientation.x;
        ocm.orientation.y = start_pose.orientation.y;
        ocm.orientation.z = start_pose.orientation.z;
        ocm.orientation.w =  start_pose.orientation.w;
		ocm.absolute_x_axis_tolerance = 0.1;
		ocm.absolute_y_axis_tolerance = 0.1;
		ocm.absolute_z_axis_tolerance = 0.1;
		ocm.weight = 0.5;

		//Now, set it as the path constraint for the group.
		moveit_msgs::Constraints test_constraints;
		test_constraints.orientation_constraints.push_back(ocm);
		move_group.setPathConstraints(test_constraints);
	
	    //Get the current state and set it as start state -- (For Actual Hardware)
		//robot_state::RobotState start_state(*move_group.getCurrentState());
	    //move_group.setStartState(start_state);
     
		geometry_msgs::Pose target_pose;
		
		if(packet1 == 2)
		{
		 target_pose.position.x = x;
		 target_pose.position.y = y;
	    }
	    else
	    {
			target_pose.position.x =   start_pose.position.x+x; 
			target_pose.position.y =   start_pose.position.y+y; 
		}
		
		std::cout<<"target_pose.position.x"<<target_pose.position.x;
		std::cout<<"target_pose.position.y"<<target_pose.position.y;
		
		
		target_pose.position.z =   start_pose.position.z+z; 
		target_pose.orientation.x =  start_pose.orientation.x;
        target_pose.orientation.y = start_pose.orientation.y;
        target_pose.orientation.z = start_pose.orientation.z;
        target_pose.orientation.w = start_pose.orientation.w;
		
		move_group.setPoseTarget(target_pose);
		move_group.setPlanningTime(5.0);
			
		success = move_group.plan(my_plan);
	
   		ROS_INFO_NAMED("Visualizing orientation (constraints) %s", success ? "" : "FAILED");
    
		// Visualize the plan in Rviz
		visual_tools.deleteAllMarkers();
		visual_tools.publishAxisLabeled(start_pose, "start");
		visual_tools.publishAxisLabeled(target_pose, "goal");
		visual_tools.publishText(text_pose, "Orientation constrained Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
  
 
		//Executing in real robot
		move_group.move(); 
		
  
		// Clearing path constraint
		move_group.clearPathConstraints();
		
		//Mark the end of execution
		subscriber_check=false;
		continuous_command = true; 
	    }
		break;
	    
	    case 0:
	    {
					  
		  //Create a quaternion to assign the start pose orientation 
		  tf::Quaternion q1(start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w);
		  
		  //Convert the quaternion into Euler angles 		 
		  tf::Matrix3x3 m(q1);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          std::cout << "IRoll: " << roll << ", IPitch: " << pitch << ", IYaw: " << yaw << std::endl;
           
          //Increment the angles from the packet data 
          alpha = alpha +roll;
          beta = beta+ pitch;
          gama = gama +yaw;
      
          //Create a quaternion for assigning final orientation 
		  tf::Quaternion q2;
		  q2 = tf::createQuaternionFromRPY(alpha, beta, gama);
		  std::cout<<"q2.x()"<<q2.x()<<std::endl;
		  std::cout<<"q2.y()"<<q2.y()<<std::endl;
		  std::cout<<"q2.z()"<<q2.z()<<std::endl;
		 		 
		 	 
          //Apply the corresponding incrementation
		  geometry_msgs::Pose target_pose;
		  target_pose.position.x = start_pose.position.x+x;
		  target_pose.position.y = start_pose.position.y+y;
		  target_pose.position.z =   start_pose.position.z+z; 
		  target_pose.orientation.x =  q2.x();
		  target_pose.orientation.y = q2.y();
		  target_pose.orientation.z = q2.z();
		  target_pose.orientation.w =  q2.w();
		  
		  				  
		  //Creating a plan object
		  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		  
		  move_group.setPoseTarget(target_pose);
		  move_group.setPlanningTime(5.0);
   
		  //planning to the corresponding setpose target
		   success = move_group.plan(my_plan);
     
		  ROS_INFO_NAMED("Visualizing plan (unconstrained goal) %s", success ? "" : "FAILED");

		  // Visualizing plans
          // ^^^^^^^^^^^^^^^^^
          // We can also visualize the plan as a line with markers in Rviz.
          ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
		  visual_tools.publishAxisLabeled(start_pose, "Unconstrained_pose");
		  visual_tools.publishText(text_pose, "Start_pose", rvt::WHITE, rvt::XLARGE);
          visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
          visual_tools.trigger();

     	   //Executing in the real hardware
           move_group.move(); 
           
          //Mark the end of execution
		  subscriber_check=false;
		  continuous_command = true; 
          }
		  break;
		  
		case 2:
		{
		  //Create a quaternion to assign the start pose orientation 
		  tf::Quaternion q1(start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w);
		  
		  //Convert the quaternion into Euler angles 		 
		  tf::Matrix3x3 m(q1);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          std::cout << "IRoll: " << roll << ", IPitch: " << pitch << ", IYaw: " << yaw << std::endl;
           
          //Increment the angles from the packet data 
          alpha = alpha +roll;
          beta = beta+ pitch;
          gama = gama +yaw;
      
          //Create a quaternion for assigning final orientation 
		  tf::Quaternion q2;
		  q2 = tf::createQuaternionFromRPY(alpha, beta, gama);
		  std::cout<<"q2.x()"<<q2.x()<<std::endl;
		  std::cout<<"q2.y()"<<q2.y()<<std::endl;
		  std::cout<<"q2.z()"<<q2.z()<<std::endl;
	     		  
		  //Set Position constraints
		  moveit_msgs::PositionConstraint pcm;
	      pcm.link_name = "lnt_gripper_tool_frame";
	      pcm.header.frame_id = "base_link";
          pcm.target_point_offset.x = start_pose.position.x;
	      pcm.target_point_offset.y =  start_pose.position.y;
	      pcm.target_point_offset.z =  start_pose.position.z; 
	      pcm.weight = 0.9;
	
	      moveit_msgs::Constraints pos_constraints;
	      pos_constraints.position_constraints.push_back(pcm);
	      move_group.setPathConstraints(pos_constraints);
	          
		  
		  geometry_msgs::Pose target_pose;
		  target_pose.position.x = start_pose.position.x;
		  target_pose.position.y = start_pose.position.y;
		  target_pose.position.z =   start_pose.position.z; 
		  target_pose.orientation.x =  q2.x();
		  target_pose.orientation.y = q2.y();
		  target_pose.orientation.z = q2.z();
		  target_pose.orientation.w = q2.w();
		  
		  				  
		  //Creating a plan object
		  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		  
		  move_group.setPoseTarget(target_pose);
		  move_group.setPlanningTime(5.0);
   
		  //planning to the corresponding setpose target
		  success = move_group.plan(my_plan);
     
		  ROS_INFO_NAMED("Visualizing plan 1 (pose constrained goal) %s", success ? "" : "FAILED");

   	      //Executing in the real hardware
          move_group.move(); 
          
   		   //Mark the end of execution
     	    subscriber_check=false;
	    	continuous_command = true; 
         }
         break;
		
		default:
		  std::cout<<"Invalid input";
	  }  
	 }
	
	}
	else
	  std::cout<<"Invalid Input";
}

}
  ros::shutdown();
  return 0;
}
