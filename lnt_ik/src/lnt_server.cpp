#include "ros/ros.h"
#include "lnt_ik/lnt_ik.h"

class lnt_control
{
 public:
	bool individual_joint_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res);
}; 

bool lnt_control::individual_joint_control(lnt_ik::lnt_ik::Request& req,lnt_ik::lnt_ik::Response& res)
{
	res.result =1;
	ROS_INFO("  sending back response: [%d]", res.result);
	return true;
  }


int main(int argc, char **argv)
{
	ros::init(argc, argv, "lnt_manipulator_server");
        ros::NodeHandle n;
	
	//Creating an object of the class
	lnt_control arm;
	ros::ServiceServer service =  n.advertiseService("lnt_server", &lnt_control::individual_joint_control, &arm);
	ros::spin();
	return 0;
}

