#include "tiago_controller/joint_controller.hpp"

namespace tiago_controller
{
  bool JointController::init(
      hardware_interface::PositionJointInterface *position_iface,
      hardware_interface::VelocityJointInterface *velocity_iface,
      ros::NodeHandle &root_nh, ros::NodeHandle &control_nh)
  {
    ROS_INFO("LOADING TIAGO JOINT CONTROLLER");

   

    return true;
  }

  bool JointController::initRequest(hardware_interface::RobotHW *robot_hw,
                                                      ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh, ClaimedResources &claimed_resources)
  {
    ROS_INFO("INIT REQUEST");

  
    return true;
  }

  /*std::string JointController::getHardwareInterfaceType() const {
  ROS_ERROR("trying to get hardware interface type tiago_controller");
  return "tiago_controller";
}*/

  void JointController::update(const ros::Time &time, const ros::Duration &period)
  {

  }

  void JointController::starting(const ros::Time &time)
  {
    ROS_INFO("Starting controller tiago_controller");
  }

  void JointController::stopping(const ros::Time &time)
  {
    ROS_INFO("Stopping controller tiago_controller");
  }
}