#include <ros/ros.h>
#include <ras_group8_arm_controller/ArmController.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_arm_controller");
  ros::NodeHandle node_handle("~");

  ras_group8_arm_controller::ArmController arm_controller(node_handle);

  ros::spin();
  return 0;
}
