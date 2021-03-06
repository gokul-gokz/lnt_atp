/*********************************************************************
   Desc:   Helper ros_control hardware interface that loads configurations
*/

#include "../include/manipulator_control/manipulator_hw_interface.h"
#include <limits>

// ROS parameter loading
//#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace manipulator_control
{
ManipulatorHWInterface::ManipulatorHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : name_("manipulator_hw_interface")
  , nh_(nh)
  , use_rosparam_joint_limits_(false)
  , use_soft_limits_if_available_(false)
{
	// object for dxl_driver.class
	dxl=new dxl_driver(nh_);
	
	// Check if the URDF model needs to be loaded
	if (urdf_model == NULL)
		loadURDF(nh, "robot_description");
	else
		urdf_model_ = urdf_model;

	if(nh_.getParam("/manipulator/joint_nums",joint_nums_)){}
	else ROS_INFO("/manipulator/joint_nums parameter could not be loaded ");
	
	//joint_names_.resize(joint_nums_.size());
	//ROS_INFO("Size of joint_names_ %d",joint_names_.size());

	for(std::map<std::string, int>::const_iterator it = joint_nums_.begin();it != joint_nums_.end(); ++it){
		//making sure i-th element of joint_names_ is key for element with value i in joint_nums_
		joint_names_.push_back(it->first);		
		//joint_names_.insert(joint_names_.begin()+it->second-1, it->first);
		
	}
	//ROS_INFO("Size of joint_names_ %d",joint_names_.size());
	for(int i=0;i<joint_names_.size();i++)
		ROS_INFO("%s ",joint_names_[i].c_str());

}

void ManipulatorHWInterface::init()
{
	// object for dxl_driver.class
	dxl->init();

  num_joints_ = joint_names_.size();
ROS_INFO("num_joints %d",num_joints_);
	

  // Status
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  // Command
  joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  joint_effort_command_.resize(num_joints_, 0.0);

  // Limits
  joint_position_lower_limits_.resize(num_joints_, 0);
  joint_position_upper_limits_.resize(num_joints_, 0);
  joint_velocity_limits_.resize(num_joints_, 0.0);
  joint_effort_limits_.resize(num_joints_, 0.0);
  //ROS_INFO("After resize");
  // Initialize interfaces for each joint
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
  	//ROS_INFO("Inside for loop");
    //ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);
	ROS_INFO("Loading joint name: %s", joint_names_[joint_id].c_str());
	//num corresponds to the number assigned to a joint with joint_nums parameter inside manipulator.yaml 
    num=joint_nums_[joint_names_[joint_id]]-1;
    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[joint_id], &joint_position_[num], &joint_velocity_[num], &joint_effort_[num]));

    // Add command interfaces to joints
    // TODO: decide based on transmissions?
    hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[num]);
    position_joint_interface_.registerHandle(joint_handle_position);

    hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[num]);
    velocity_joint_interface_.registerHandle(joint_handle_velocity);

    hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[num]);
    effort_joint_interface_.registerHandle(joint_handle_effort);

    // Load the joint limits
    registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, num);
  }  // end for each joint
	//ROS_INFO("Outside for loop");

  registerInterface(&joint_state_interface_);     // From RobotHW base class.
  registerInterface(&position_joint_interface_);  // From RobotHW base class.
  registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
  registerInterface(&effort_joint_interface_);    // From RobotHW base class.

  ROS_INFO_STREAM_NAMED(name_, "ManipulatorHWInterface Ready.");
}

void ManipulatorHWInterface::read(ros::Duration &elapsed_time){
	ROS_INFO("ManipulatorHWInterface::read called");
	
	//count+=100;
	//joint_position_[0]=count;	
	//ROS_INFO("joint_position_ is ");
	//printf("Joint position %G ",joint_position_[0]);
	//for(int i=0;i<joint_position_.size();i++)
		//ROS_INFO("%G ",joint_position_[i]);
	dxl->read(joint_position_,joint_velocity_,joint_effort_);
	
	
}

void ManipulatorHWInterface::write(ros::Duration &elapsed_time){
	ROS_INFO("ManipulatorHWInterface::read called");
	//ROS_INFO("Size of joint_position_command_ %d",joint_position_command_.size());
	//for(int i=0;i<joint_position_command_.size();i++)
		//ROS_INFO("%G ",joint_position_command_[i]);
	dxl->write(joint_position_command_,joint_velocity_command_,joint_effort_command_);

	
}






void ManipulatorHWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                                             const hardware_interface::JointHandle &joint_handle_velocity,
                                             const hardware_interface::JointHandle &joint_handle_effort,
                                             std::size_t joint_id)
{
  // Default values
  joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
  joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

  // Limits datastructures
  joint_limits_interface::JointLimits joint_limits;     // Position
  joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
  bool has_joint_limits = false;
  bool has_soft_limits = false;

  // Get limits from URDF
  if (urdf_model_ == NULL)
  {
    ROS_WARN_STREAM_NAMED(name_, "No URDF model loaded, unable to get joint limits");
    return;
  }

  // Get limits from URDF
  urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

  // Get main joint limits
  if (urdf_joint == NULL)
  {
    ROS_ERROR_STREAM_NAMED(name_, "URDF joint not found " << joint_names_[joint_id]);
    return;
  }

  // Get limits from URDF
  if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
  {
    has_joint_limits = true;
    ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF position limits ["
                                                            << joint_limits.min_position << ", "
                                                            << joint_limits.max_position << "]");
    if (joint_limits.has_velocity_limits)
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
                                                              << joint_limits.max_velocity << "]");
  }
  else
  {
    if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have a URDF "
                            "position limit");
  }

  // Get limits from ROS param
  if (use_rosparam_joint_limits_)
  {
    if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits))
    {
      has_joint_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Joint " << joint_names_[joint_id] << " has rosparam position limits ["
                                      << joint_limits.min_position << ", " << joint_limits.max_position << "]");
      if (joint_limits.has_velocity_limits)
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
                                                                << " has rosparam velocity limit ["
                                                                << joint_limits.max_velocity << "]");
    }  // the else debug message provided internally by joint_limits_interface
  }

  // Get soft limits from URDF
  if (use_soft_limits_if_available_)
  {
    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
    {
      has_soft_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has soft joint limits.");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have soft joint "
                             "limits");
    }
  }

  // Quit we we haven't found any limits in URDF or rosparam server
  if (!has_joint_limits)
  {
    return;
  }

  // Copy position limits if available
  if (joint_limits.has_position_limits)
  {
    // Slighly reduce the joint limits to prevent floating point errors
    joint_limits.min_position += std::numeric_limits<double>::epsilon();
    joint_limits.max_position -= std::numeric_limits<double>::epsilon();

    joint_position_lower_limits_[joint_id] = joint_limits.min_position;
    joint_position_upper_limits_[joint_id] = joint_limits.max_position;
  }

  // Copy velocity limits if available
  if (joint_limits.has_velocity_limits)
  {
    joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
  }

  // Copy effort limits if available
  if (joint_limits.has_effort_limits)
  {
    joint_effort_limits_[joint_id] = joint_limits.max_effort;
  }

  if (has_soft_limits)  // Use soft limits
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Using soft saturation limits");
    const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position,
                                                                                       joint_limits, soft_limits);
    pos_jnt_soft_limits_.registerHandle(soft_handle_position);
    const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity,
                                                                                       joint_limits, soft_limits);
    vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);
    const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits,
                                                                                   soft_limits);
    eff_jnt_soft_limits_.registerHandle(soft_handle_effort);
  }
  else  // Use saturation limits
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Using saturation limits (not soft limits)");

    const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
    pos_jnt_sat_interface_.registerHandle(sat_handle_position);

    const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
    vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);

    const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
    eff_jnt_sat_interface_.registerHandle(sat_handle_effort);
  }
}

void ManipulatorHWInterface::reset()
{
  // Reset joint limits state, in case of mode switch or e-stop
  pos_jnt_sat_interface_.reset();
  pos_jnt_soft_limits_.reset();
}

void ManipulatorHWInterface::printState()
{
  // WARNING: THIS IS NOT REALTIME SAFE
  // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
  ROS_INFO_STREAM_THROTTLE(1, std::endl
                                  << printStateHelper());
}

std::string ManipulatorHWInterface::printStateHelper()
{
  std::stringstream ss;
  std::cout.precision(15);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ss << "j" << i << ": " << std::fixed << joint_position_[i] << "\t ";
    ss << std::fixed << joint_velocity_[i] << "\t ";
    ss << std::fixed << joint_effort_[i] << std::endl;
  }
  return ss.str();
}

std::string ManipulatorHWInterface::printCommandHelper()
{
  std::stringstream ss;
  std::cout.precision(15);
  ss << "    position     velocity         effort  \n";
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ss << "j" << i << ": " << std::fixed << joint_position_command_[i] << "\t ";
    ss << std::fixed << joint_velocity_command_[i] << "\t ";
    ss << std::fixed << joint_effort_command_[i] << std::endl;
  }
  return ss.str();
}

void ManipulatorHWInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
  std::string urdf_string;
  urdf_model_ = new urdf::Model();

  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok())
  {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
    	
      ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                            nh.getNamespace() << search_param_name);
      nh.getParam(search_param_name, urdf_string);
	//ROS_INFO("Inside ManipulatorHWInterface::loadURDF if loop %s",urdf_string.c_str());
    }
    else
    {
    	ROS_INFO("Inside ManipulatorHWInterface::loadURDF else loop");
      ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                            nh.getNamespace() << param_name);
      nh.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }

  if (!urdf_model_->initString(urdf_string))
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
  else
    ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
}

} // namesp
