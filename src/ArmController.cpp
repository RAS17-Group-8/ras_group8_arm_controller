#include <ras_group8_arm_controller/ArmController.hpp>

namespace ras_group8_arm_controller {

ArmController::ArmController(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  arm_cartesian_up_subscriber_ = node_handle_.subscribe(arm_cartesian_up_topic_, 1,
                                      &ArmController::uarmDesiredPositionUpCallback, this);

  arm_cartesian_down_subscriber_ = node_handle_.subscribe(arm_cartesian_down_topic_, 1,
                                      &ArmController::uarmDesiredPositionDownCallback, this);

  arm_status_subscriber_ = node_handle_.subscribe(arm_status_topic_, 1,
                                      &ArmController::uarmJointStateCallback, this);

  arm_move_client_= node_handle_.serviceClient<uarm::MoveToJoints>(arm_joint_topic_);
  arm_pump_client_= node_handle_.serviceClient<uarm::Pump>(arm_pump_topic_);

  ROS_INFO("Successfully launched node.");
}

ArmController::~ArmController()
{
}

bool ArmController::readParameters()
{
    if (!node_handle_.getParam("arm_joint_topic", arm_joint_topic_))
      return false;
    ROS_INFO("P: arm_joint_topic_ = %s", arm_joint_topic_.c_str());

    if (!node_handle_.getParam("arm_status_topic", arm_status_topic_))
      return false;
    ROS_INFO("P: arm_status_topic_ = %s", arm_status_topic_.c_str());

    if (!node_handle_.getParam("arm_pump_topic", arm_pump_topic_))
      return false;
    ROS_INFO("P: arm_pump_topic_= %s", arm_pump_topic_.c_str());

    if (!node_handle_.getParam("arm_cartesian_up_topic", arm_cartesian_up_topic_))
      return false;
    ROS_INFO("P: arm_cartesian_up_topic_ = %s", arm_cartesian_up_topic_.c_str());

    if (!node_handle_.getParam("arm_cartesian_down_topic", arm_cartesian_down_topic_))
      return false;
    ROS_INFO("P: arm_cartesian_down_topic_ = %s", arm_cartesian_down_topic_.c_str());

    if (!node_handle_.getParam("arm_dim/a", arm_a))
      return false;
    ROS_INFO("P: arm_a = %f", arm_a);

    if (!node_handle_.getParam("arm_dim/b", arm_b))
      return false;
    ROS_INFO("P: arm_b = %f", arm_b);

    if (!node_handle_.getParam("arm_dim/c", arm_c))
      return false;
    ROS_INFO("P: arm_c = %f", arm_c);

    if (!node_handle_.getParam("arm_dim/d", arm_d))
      return false;
    ROS_INFO("P: arm_d = %f", arm_d);

    if (!node_handle_.getParam("arm_dim/e", arm_e))
      return false;
    ROS_INFO("P: arm_e = %f", arm_e);

    if (!node_handle_.getParam("arm_dim/f", arm_f))
      return false;
    ROS_INFO("P: arm_f = %f", arm_f);

    if (!node_handle_.getParam("arm_angles/j0_zero", arm_j0_zero))
      return false;
    ROS_INFO("P: arm_j0_zero = %f", arm_j0_zero);

    if (!node_handle_.getParam("arm_angles/j1_zero", arm_j1_zero))
      return false;
    ROS_INFO("P: arm_j1_zero = %f", arm_j1_zero);

    if (!node_handle_.getParam("arm_angles/j2_v", arm_j2_v))
      return false;
    ROS_INFO("P: arm_j2_v = %f", arm_j2_v);

    if (!node_handle_.getParam("arm_base_position/x", base_position_global.x))
      return false;
    ROS_INFO("P: base_position_global.x = %f", base_position_global.x);

    if (!node_handle_.getParam("arm_base_position/y", base_position_global.y))
      return false;
    ROS_INFO("P: base_position_global.y = %f", base_position_global.y);

    if (!node_handle_.getParam("arm_base_position/z", base_position_global.z))
      return false;
    ROS_INFO("P: base_position_global.z= %f",base_position_global.z);

    if (!node_handle_.getParam("arm_movement/move_time", move_time))
      return false;
    ROS_INFO("P: move_time= %i",move_time);

    return true;
}


void ArmController::uarmJointStateCallback(const sensor_msgs::JointState &msg)
{ //transfer the message from the servos into a global position into the armframe 

    ROS_INFO("Actual Position ArmPostion: alpha:%f, beta:%f, angle servo1:%f, gamma",actual_position_global.x,actual_position_global.y,actual_position_global.z);
    double actual_position_w;
    double actual_position_alpha_d;
    double actual_position_beta_d;
    double actual_position_gamma_d;

    // use the calibration of the servos to get the real position
    actual_position_alpha_d=(double) msg.position[1]-arm_j1_zero;
    actual_position_beta_d=(double) (270-actual_position_alpha_d-msg.position[2]+arm_j2_v);
    actual_position_gamma_d=(double) msg.position[0]-arm_j0_zero;

    ROS_INFO("Actual Position ArmPostion: alpha:%f, beta:%f, gamma:%f", actual_position_alpha_d,actual_position_beta_d,actual_position_gamma_d);

    // change the angle in rad
    actual_position_alpha=actual_position_alpha_d*M_PI/180;
    actual_position_beta=actual_position_beta_d*M_PI/180;
    actual_position_gamma=actual_position_gamma_d*M_PI/180;

    actual_position_w=arm_b+arm_c*cos(actual_position_alpha)+arm_d*cos(actual_position_alpha+actual_position_beta)+arm_e;

    actual_position_global.x=actual_position_w*sin(actual_position_gamma);
    actual_position_global.y=actual_position_w*cos(actual_position_gamma);

    actual_position_global.z=arm_a+arm_c*sin(actual_position_alpha)+arm_d*sin(actual_position_alpha+actual_position_beta)+arm_f;

    ROS_INFO("Actual Position Arm_Fram in cm: X: %f, Y: %f, Z: %f",actual_position_global.x,actual_position_global.y,actual_position_global.z);
}

void ArmController::uarmDesiredPositionUpCallback(const geometry_msgs::Vector3 &msg)
{ //transfer the message about the desired coordinates in the Armframe into a command to the servos (joint space)
  if(msg.x>10||msg.x<-10||msg.y>30||msg.y<9.5||msg.z>30||msg.z<-16.5)
  {
    ROS_INFO("This global postion X: %f, Y: %f, Z: %f is not allowed",msg.x,msg.y,msg.z);
  }
  else
  {
      uarmMoveToCoordinates(msg);

      pump_message.request.pump_status=true;
      arm_pump_client_.call(pump_message);

      uarmMoveToCoordinates(base_position_global);
  }
}

void ArmController::uarmDesiredPositionDownCallback(const geometry_msgs::Vector3 &msg)
{ //transfer the message about the desired coordinates in the Armframe into a command to the servos (joint space)
  if(msg.x>10||msg.x<-10||msg.y>30||msg.y<9.5||msg.z>30||msg.z<-16.5)
  {
    ROS_INFO("This global postion X: %f, Y: %f, Z: %f is not allowed",msg.x,msg.y,msg.z);
  }
  else
  {
      uarmMoveToCoordinates(msg);

      pump_message.request.pump_status=false;
      arm_pump_client_.call(pump_message);
  }
}

void ArmController::uarmMoveToCoordinates(const geometry_msgs::Vector3 &msg)
{
    double desired_position_w_star;
    double desired_position_z_star;

    double desired_position_alpha_help;

    double desired_joint_servo_0;
    double desired_joint_servo_1;
    double desired_joint_servo_2;

    double desired_joint_space_beta;

    desired_position_w_star=sqrt(pow(msg.x,2)+pow(msg.y,2))-(arm_b+arm_e);
    desired_position_z_star=msg.z-(arm_a+arm_f);


    desired_position_alpha_help=acos((pow(arm_c,2)+pow(desired_position_w_star,2)+pow(desired_position_z_star,2)-pow(arm_d,2))/(2*arm_c*sqrt(pow(desired_position_w_star,2)+pow(desired_position_z_star,2))));
    desired_position_alpha=desired_position_alpha_help+atan2(desired_position_z_star,desired_position_w_star);
    desired_position_beta=M_PI+acos((pow(arm_d,2)+pow(arm_c,2)-pow(desired_position_z_star,2)-pow(desired_position_w_star,2))/(2*arm_c*arm_d));
    desired_position_gamma=atan2(msg.x,msg.y);
    //ROS_INFO("Desired Position Calculation frame: Alpha: %f, Beta: %f, Gamma: %f",desired_position_alpha, desired_position_beta, desired_position_gamma);

    desired_joint_servo_0=desired_position_gamma*180/M_PI;
    desired_joint_servo_1=desired_position_alpha*180/M_PI;
    desired_joint_space_beta=desired_position_beta*180/M_PI;
    desired_joint_servo_2=desired_joint_space_beta-270+desired_joint_servo_1;

    ROS_INFO("Desired Postion Caltulation frame: Alpha:%f, Beta:%f, Postion Servo 2:%f, Gamma:%f ",desired_joint_servo_1,desired_joint_space_beta, desired_joint_servo_2,desired_joint_servo_0);

    // use the arm calibration
    if ((desired_joint_space_message.request.j0 = desired_joint_servo_0+arm_j0_zero)>360)
        desired_joint_space_message.request.j0=desired_joint_space_message.request.j0-360;

    if ((desired_joint_space_message.request.j1 = desired_joint_servo_1+arm_j1_zero)>360)
         desired_joint_space_message.request.j1=desired_joint_space_message.request.j1-360;

    if ((desired_joint_space_message.request.j2 = arm_j2_v-desired_joint_servo_2)>360)
        desired_joint_space_message.request.j2=desired_joint_space_message.request.j2-360;

    desired_joint_space_message.request.j3 = 0;
    desired_joint_space_message.request.move_mode=0;
    desired_joint_space_message.request.interpolation_type=2;
    desired_joint_space_message.request.movement_duration.sec=move_time;

    ////////////////////give the angle of the sevos in the calibrationframe in rad//////////////

    ROS_INFO("Desired Position Servo0: %f, Servo1: %f, Servo2: %f",desired_joint_space_message.request.j0 ,desired_joint_space_message.request.j1 ,desired_joint_space_message.request.j2);

    if (1==2) // insert the forbidden postions in joint coordinates
    {
        ROS_INFO("The joint position j0=%f, j1=%f, j2=%f is not allowed",desired_joint_servo_0,desired_joint_servo_1,desired_joint_servo_2);
    }
    else
    {
        arm_move_client_.call(desired_joint_space_message);

        if(desired_joint_space_message.response.error)
        {
            ROS_INFO("Movement was not possible");
        }
        else
        {
           ROS_INFO("Actual Position Servo0: %f, Servo1: %f, Servo2: %f",desired_joint_space_message.response.j0 ,desired_joint_space_message.response.j1 ,desired_joint_space_message.response.j2);
        }
    }
}

} /* namespace */
