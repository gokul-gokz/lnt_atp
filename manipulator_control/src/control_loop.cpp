/* Author: Dave Coleman
   Desc:   Example control loop for reading, updating, and writing commands to a hardware interface
   using MONOTOIC system time
*/

#include "../include/manipulator_control/control_loop.h"
// ROS parameter loading
//#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace manipulator_control
{
HWControlLoop::HWControlLoop(
    ros::NodeHandle& nh, boost::shared_ptr<manipulator_control::ManipulatorHWInterface> hardware_interface)
  : nh_(nh), hardware_interface_(hardware_interface)
{
	string name_ = "hw_control_loop";
	// Create the controller manager
	controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

	if(nh_.getParam("/control_loop/loop_hz",loop_hz_)){}
	else ROS_INFO("/control_loop/loop_hz parameter could not be loaded ");

	if(nh_.getParam("/control_loop/cycle_time_error_threshold",cycle_time_error_threshold_)){}
	else ROS_INFO("/control_loop/cycle_time_error_threshold parameter could not be loaded ");

	// Get current time for use with first update
	clock_gettime(CLOCK_MONOTONIC, &last_time_);

	desired_update_period_ = ros::Duration(1 / loop_hz_);
}

void HWControlLoop::run()
{
  ros::Rate rate(loop_hz_);
  while(ros::ok()) {
    update();
    rate.sleep();
  }
}

void HWControlLoop::update()
{
  // Get change in time
  clock_gettime(CLOCK_MONOTONIC, &current_time_);
  elapsed_time_ =
      ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
  last_time_ = current_time_;
  // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "generic_hw_main","Sampled update loop with elapsed
  // time " << elapsed_time_.toSec());

  // Error check cycle time
  const double cycle_time_error = (elapsed_time_ - desired_update_period_).toSec();
  if (cycle_time_error > cycle_time_error_threshold_)
  {
    ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                     << cycle_time_error << ", cycle time: " << elapsed_time_
                                     << ", threshold: " << cycle_time_error_threshold_);
  }

  // Input
  hardware_interface_->read(elapsed_time_);

  // Control
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  hardware_interface_->write(elapsed_time_);
}

} // namespace
