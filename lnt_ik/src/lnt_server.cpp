#include "ros/ros.h"
#include "lnt_ik/lnt_ik.h"

//Moveit header files
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

//For mathematical calculation and conversion
#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_datatypes.h>

class lnt_control
{
 private:
           
	//Variable for storing the planning group which we want
	static constexpr const char* PLANNING_GROUP = "manipulator";  //static cannot be used for string data member and so using char* 
	
	//Pointer object for MoveGroupInterface class
	moveit::planning_interface::MoveGroupInterface *move_group;
	
	//Pointer variable for storing state(position) of joint model group	
	const robot_state::JointModelGroup *joint_model_group;
	
	//Create an object for storing the curret state of the robot
	moveit::core::RobotStatePtr current_state;  
	
	//Temporary variable for copying the joint group positions
	std::vector<double> joint_group_positions;  
	
	//variable for storing safety limits for each joints
	double max[6] = {3.14,3.14,0,2.09,1.57,3.14};
	double min[6] = {-3.14,0,-3.14,-2.09,-1.57,-3.14};
	
	//Angle converted to radians and stored
	float position;
	
	//Variable for storing the position of multiple joints
	float multiple_joint_position[6];
  	

 public:
	bool individual_joint_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res);
	
	bool multiple_joint_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res);
	
	bool safety_check(int joint, double position);
	
	float Deg_to_Rad(float angle);
	
	lnt_control();
}; 

//Class constructor
lnt_control::lnt_control()
{
	//Invoke the movegroup interface for the planning group which we want
	  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
 }
 
 //Safety limit check function
bool lnt_control::safety_check(int joint, double position)
{
  if ((position>=min[joint]) && (position<=max[joint]))
    return true;
  else
    return false;
}

float lnt_control::Deg_to_Rad(float angle)
{
	 angle = (angle *3.14)/180;
	 ROS_INFO("Angle=%f",angle);
	 return angle;
}



bool lnt_control::individual_joint_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res)
{
  position = lnt_control::Deg_to_Rad(req.values[1]);
  if(lnt_control::safety_check(req.values[0],position))
  {
	 //Store the current state information(position/accleration/velocity) from the movegroup member function
      current_state = move_group->getCurrentState();
      
      //Get the current position of the joints of the corresponding planning group
      joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
  
      //Copy the positions to another variable for modification.
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // Modify the joint state accordingly
      joint_group_positions[req.values[0]] = position;  // radians
      move_group->setJointValueTarget(joint_group_positions);
      
      //Create a plan object(local member) 
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = move_group->plan(my_plan);

      //Execute the plan
      move_group->move(); 

      res.result = success;

      ROS_INFO("Sending back response: [%d]", res.result);
      return true;
  }
  else
  {
	  ROS_INFO("Exceeding joint limits");
      return false;
  }
}

bool lnt_control::multiple_joint_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res)
{
	
   //Store the current state information(position/accleration/velocity) from the movegroup member function
   current_state = move_group->getCurrentState();
      
   //Get the current position of the joints of the corresponding planning group
   joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
  
   //Copy the positions to another variable for modification.
   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  //Modify the joint state accordingly
  for(int i=0;i<6;i++)
  {
	//Degree to radians conversion
	multiple_joint_position[i] = lnt_control::Deg_to_Rad(req.values[i]);
	
	//Checking the limits
	if(safety_check(i,multiple_joint_position[i]))
	 {	  
        joint_group_positions[i] = multiple_joint_position[i];
	   }
	   else
	   {
		   std::cout<<"Joint"<<i<<" out of range"<<std::endl;
       }
     }  
      
      move_group->setJointValueTarget(joint_group_positions);
      
      //Create a plan object(local member) 
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = move_group->plan(my_plan);

      //Execute the plan
      move_group->move(); 

      res.result = success;

      ROS_INFO("Joint space goal(Multiple_joints): [%d]", res.result);
      return true;
	
	
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "lnt_manipulator_server");
        ros::NodeHandle n;

	//Creating an object of the class
	lnt_control arm;
	ros::ServiceServer service1 =  n.advertiseService("joint_space_control_individual", &lnt_control::individual_joint_control, &arm);
	ros::ServiceServer service2 =  n.advertiseService("joint_space_control_multiple", &lnt_control::multiple_joint_control, &arm);
	ros::AsyncSpinner spinner(3);
  	spinner.start();
	ros::waitForShutdown();
	return 0;
}

