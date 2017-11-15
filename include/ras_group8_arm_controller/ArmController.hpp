#pragma once

#include <ros/ros.h>

#include <string.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <uarm/MoveToJoints.h>
#include <uarm/Pump.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include "ras_group8_arm_controller/MoveArm.h"

namespace ras_group8_arm_controller {

class ArmController
{
public:
  ArmController(ros::NodeHandle& node_handle);
  virtual ~ArmController();

private:
  bool readParameters();
  void uarmJointStateCallback(const sensor_msgs::JointState &msg);
  bool uarmDesiredPositionUp(ras_group8_arm_controller::MoveArm::Request &req,
                             ras_group8_arm_controller::MoveArm::Response &res);
  bool uarmDesiredPositionDown(ras_group8_arm_controller::MoveArm::Request &req,
                               ras_group8_arm_controller::MoveArm::Response &res);
  bool uarmMoveToCoordinates(const geometry_msgs::Point &msg);


  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;

  ros::ServiceServer arm_cartesian_up_server_;
  ros::ServiceServer arm_cartesian_down_server_;
  ros::Subscriber arm_status_subscriber_;

  //ros::Publisher arm_move_publisher_;
  ros::ServiceClient  arm_move_client_;
  ros::ServiceClient  arm_pump_client_;

  
  /* Parameters
   */

  std::string arm_joint_topic_;
  std::string arm_status_topic_;
  std::string arm_cartesian_up_topic_;
  std::string arm_cartesian_down_topic_;
  std::string arm_pump_topic_;

  // variables

  geometry_msgs::Point actual_position_global;
  geometry_msgs::Point base_position_global;
  uarm::MoveToJoints desired_joint_space_message;
  uarm::Pump pump_message;



  double desired_position_alpha;
  double desired_position_beta;
  double desired_position_gamma;

  double actual_position_alpha;
  double actual_position_beta;
  double actual_position_gamma;


  double arm_a;
  double arm_b;
  double arm_c;
  double arm_d;
  double arm_e;
  double arm_f;

  double arm_j0_zero;
  double arm_j1_zero;
  double arm_j2_v;

  int move_time;


};

} /* namespace */
