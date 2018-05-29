#include "../include/manipulator_control/dxl_driver.h"

int main(int argc, char** argv){
ros::init(argc, argv, "dxl_test_main");
ros::NodeHandle nh;

std::vector<double> joint_position_;
std::vector<double> joint_velocity_;
std::vector<double> joint_effort_;

std::vector<double> joint_position_c;
std::vector<double> joint_velocity_c;
std::vector<double> joint_effort_c;



joint_position_.resize(8, 0.0);
joint_velocity_.resize(8, 0.0);
joint_effort_.resize(8, 0.0);

joint_position_c.resize(8, 0.0);
joint_velocity_c.resize(8, 0.0);
joint_effort_c.resize(8, 0.0);


dxl_driver d(nh);
d.init();

//d.read(joint_position_,joint_velocity_,joint_effort_);

printf("joint position c");
for (int i = 0;i<2;i++){
joint_position_c[i]=2.0;
}

d.write(joint_position_c,joint_velocity_c,joint_effort_c);

printf("joint position ");
for (int i = 0;i<2;i++){
printf("%G ",joint_position_[i]);
}

d.read(joint_position_,joint_velocity_,joint_effort_);

return 0;
}
