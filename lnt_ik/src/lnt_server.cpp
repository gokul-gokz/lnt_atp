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
	
	bool cartesian_space_unconstrained_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res);
	
	bool cartesian_space_orientation_constrained_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res);
	
	bool cartesian_space_position_constrained_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res);
	
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

bool lnt_control::cartesian_space_unconstrained_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res)
{
	//Create local variables for assigning the packet data
	float x,y,z,alpha,beta,gama;
	
	//Assign the values from the client request data
	x=req.values[0];
	y=req.values[1];
	z=req.values[2];
	alpha=req.values[3];
	beta=req.values[4];
	gama=req.values[5];
	
	
	//Creating a local variable for storing the start_pose
	geometry_msgs::Pose start_pose;
	
	//Get the current pose
	geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
	
	//Copy the current pose to start pose evariable
	start_pose.position.x = current_pose.pose.position.x;
    start_pose.position.y = current_pose.pose.position.y;
    start_pose.position.z = current_pose.pose.position.z; 
    start_pose.orientation.x =  current_pose.pose.orientation.x;
    start_pose.orientation.y = current_pose.pose.orientation.y;
    start_pose.orientation.z = current_pose.pose.orientation.z;
    start_pose.orientation.w = current_pose.pose.orientation.w;
	
	
	//Create a quaternion to assign the start pose orientation 
	tf::Quaternion q1(start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w);
		  
	//Convert the quaternion into Euler angles 		 
	tf::Matrix3x3 m(q1);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    //Increment the angles from the packet data 
    alpha = alpha +roll;
    beta = beta+ pitch;
    gama = gama +yaw;
    
    //Create a quaternion for assigning final orientation 
	tf::Quaternion q2;
	q2 = tf::createQuaternionFromRPY(alpha, beta, gama);
	
	//Apply the corresponding incrementation
	geometry_msgs::Pose target_pose;
	target_pose.position.x = start_pose.position.x+x;
	target_pose.position.y = start_pose.position.y+y;
	target_pose.position.z =   start_pose.position.z+z; 
	target_pose.orientation.x =  q2.x();
	target_pose.orientation.y = q2.y();
	target_pose.orientation.z = q2.z();
	target_pose.orientation.w =  q2.w();
	      
    //Create a plan object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    //Set the target pose
    move_group->setPoseTarget(target_pose);
    move_group->setPlanningTime(5.0);
    
    //Execute the plan
    move_group->move(); 
     
    return true;
	
}

bool lnt_control::cartesian_space_orientation_constrained_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res)
{
	
	//Create local variables for assigning the packet data
	float x,y,z,alpha,beta,gama;
	
	//Assign the values from the client request data
	x=req.values[0];
	y=req.values[1];
	z=req.values[2];
	alpha=req.values[3];
	beta=req.values[4];
	gama=req.values[5];
	
	
	//Creating a local variable for storing the start_pose
	geometry_msgs::Pose start_pose;
	
	//Get the current pose
	geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
	
	//Copy the current pose to start pose evariable
	start_pose.position.x = current_pose.pose.position.x;
    start_pose.position.y = current_pose.pose.position.y;
    start_pose.position.z = current_pose.pose.position.z; 
    start_pose.orientation.x =  current_pose.pose.orientation.x;
    start_pose.orientation.y = current_pose.pose.orientation.y;
    start_pose.orientation.z = current_pose.pose.orientation.z;
    start_pose.orientation.w = current_pose.pose.orientation.w;
    
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
	move_group->setPathConstraints(test_constraints);
	
	//Assign the target pose
	geometry_msgs::Pose target_pose;
	target_pose.position.x =   start_pose.position.x+x; 
	target_pose.position.y =   start_pose.position.y+y;
	target_pose.position.z =   start_pose.position.z+z; 
	target_pose.orientation.x =  start_pose.orientation.x;
    target_pose.orientation.y = start_pose.orientation.y;
    target_pose.orientation.z = start_pose.orientation.z;
    target_pose.orientation.w = start_pose.orientation.w; 
    
    //Create a plan object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    //Set the target
    move_group->setPoseTarget(target_pose);
	move_group->setPlanningTime(5.0);
			
	bool success = move_group->plan(my_plan);
	
	//Executing the plan
	move_group->move(); 
	  
	// Clearing path constraint
	move_group->clearPathConstraints();
	
	return true;
		
}

bool lnt_control::cartesian_space_position_constrained_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res)
{
	//Create local variables for assigning the packet data
	float x,y,z,alpha,beta,gama;
	
	//Assign the values from the client request data
	x=req.values[0];
	y=req.values[1];
	z=req.values[2];
	alpha=req.values[3];
	beta=req.values[4];
	gama=req.values[5];
	
	
	//Creating a local variable for storing the start_pose
	geometry_msgs::Pose start_pose;
	
	//Get the current pose
	geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
	
	//Copy the current pose to start pose evariable
	start_pose.position.x = current_pose.pose.position.x;
    start_pose.position.y = current_pose.pose.position.y;
    start_pose.position.z = current_pose.pose.position.z; 
    start_pose.orientation.x =  current_pose.pose.orientation.x;
    start_pose.orientation.y = current_pose.pose.orientation.y;
    start_pose.orientation.z = current_pose.pose.orientation.z;
    start_pose.orientation.w = current_pose.pose.orientation.w;
	
	
	//Create a quaternion to assign the start pose orientation 
	tf::Quaternion q1(start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w);
		  
	//Convert the quaternion into Euler angles 		 
	tf::Matrix3x3 m(q1);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    //Increment the angles from the packet data 
    alpha = alpha +roll;
    beta = beta+ pitch;
    gama = gama +yaw;
    
    //Create a quaternion for assigning final orientation 
	tf::Quaternion q2;
	q2 = tf::createQuaternionFromRPY(alpha, beta, gama);
	
	//Create  Position constraints
    moveit_msgs::PositionConstraint pcm;
	pcm.link_name = "lnt_gripper_tool_frame";
	pcm.header.frame_id = "base_link";
    pcm.target_point_offset.x = start_pose.position.x;
	pcm.target_point_offset.y =  start_pose.position.y;
	pcm.target_point_offset.z =  start_pose.position.z; 
	pcm.weight = 0.9;
	
	//Set the constraint
	moveit_msgs::Constraints pos_constraints;
	pos_constraints.position_constraints.push_back(pcm);
	move_group->setPathConstraints(pos_constraints);
	
	//set the target pose
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
	  
	//Set the target pose  
	move_group->setPoseTarget(target_pose);
	move_group->setPlanningTime(5.0);
	
	//Planto the corresponding setpose target
	bool success = move_group->plan(my_plan);
	
	//Execute the plan
	move_group->move(); 
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
	ros::ServiceServer service3 =  n.advertiseService("cartesian_space_unconstrained", &lnt_control::cartesian_space_unconstrained_control, &arm);
	ros::ServiceServer service4 =  n.advertiseService("cartesian_space_orientation_constraint", &lnt_control::cartesian_space_orientation_constrained_control, &arm);
	ros::ServiceServer service5 =  n.advertiseService("cartesian_space_position_constraint", &lnt_control::cartesian_space_position_constrained_control, &arm);
	ros::AsyncSpinner spinner(3);
  	spinner.start();
	ros::waitForShutdown();
	return 0;
}

