#include "tiago_controller/JointController.hpp"

bool tiago_controller::JointController::init(
  hardware_interface::PositionJointInterface* position_iface,
  hardware_interface::VelocityJointInterface* velocity_iface,
  ros::NodeHandle& root_nh, ros::NodeHandle& control_nh) {
  ROS_INFO("LOADING JOINT CONTROLLER");

  joint_command_sub_.reset(new ros::Subscriber(
    control_nh.subscribe<robot_framework_msgs::command>(
    "/RobotFramework/joint_command", 1,
    &tiago_controller::JointController::updateCommand, this)));

  std::string ns, paramJoints = "/tiago_controller/joints";
  std::string paramInitPos = "/tiago_controller/initial_positions";
  if (control_nh.getParam("/namespace", ns)) {
    paramJoints = ns + paramJoints;
    paramInitPos = ns + paramInitPos;
  }

  double dt = 0;
  if (control_nh.getParam("/dt", dt)) {
    dt_ = dt;
  }

  if (!control_nh.getParam(paramJoints, jNames_)) {
    std::cerr << "can't read param " << paramJoints << std::endl;
    return false;
  }

  double initPos = 0;
  for (int i = 0; i < jNames_.size(); ++i) {
    if (control_nh.getParam(paramInitPos + "/" + jNames_[i], initPos)) {
      mapQdesired_[jNames_[i]] = initPos;
    } else {
      mapQdesired_[jNames_[i]] = 0;
    }
  }

  // Get a joint handle
  for (auto &jt : jNames_) {
    bool found = false;
    try {
      joints_.push_back(position_iface->getHandle(jt));
      mapQInit_[jt] = joints_.back().getPosition();
      ROS_INFO_STREAM("Found joint '" << jt << "' as position");
      found = true;
    }
    catch (...) {
    }
    try {
      joints_.push_back(velocity_iface->getHandle(jt));
      mapQInit_[jt] = joints_.back().getVelocity();
      ROS_INFO_STREAM("Found joint '" << jt << "' as velocity");
      found = true;
    }
    catch (...) {
    }
    if (!found) {
      ROS_ERROR_STREAM("Could not find joint '" << jt <<
        " as position or velocity");
      return false;
    }
  }
  double init_duration = 5.0;

  for (double t = 0.0; t < init_duration; t += dt_) {
    for (int i = 0; i < jNames_.size(); ++i) {
      double cmd = dt_*(mapQdesired_[jNames_[i]]-mapQInit_[jNames_[i]])/init_duration + mapQInit_[jNames_[i]];
      joints_[i].setCommand(cmd);
    }
  }

  return true;
}

bool tiago_controller::JointController::initRequest(hardware_interface::RobotHW* robot_hw,
  ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, ClaimedResources&claimed_resources) {
  ROS_INFO("INIT REQUEST");

  if (state_ != CONSTRUCTED) {
    ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
    return false;
  }

  hardware_interface::PositionJointInterface* position_iface =
    robot_hw->get<hardware_interface::PositionJointInterface>();

  if (!position_iface) {
    ROS_ERROR("This controller requires a hardware interface of type PositionJointInterface."
              " Make sure this is registered in the hardware_interface::RobotHW class.");
      return false;
  }

  position_iface->clearClaims();

  hardware_interface::VelocityJointInterface* velocity_iface =
    robot_hw->get<hardware_interface::VelocityJointInterface>();

  if (!velocity_iface) {
    ROS_ERROR("This controller requires a hardware interface of type VelocityJointInterface."
              " Make sure this is registered in the hardware_interface::RobotHW class.");
      return false;
  }

  velocity_iface->clearClaims();

  if (!init(position_iface, velocity_iface, root_nh, controller_nh)) {
    ROS_ERROR("Failed to initialize the controller");
    return false;
  }

  // Saves the resources claimed by this controller
  claimed_resources.push_back(hardware_interface::InterfaceResources(
      hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>(),
      position_iface->getClaims()));
  position_iface->clearClaims();

  claimed_resources.push_back(hardware_interface::InterfaceResources(
      hardware_interface::internal::demangledTypeName<hardware_interface::VelocityJointInterface>(),
      velocity_iface->getClaims()));
  velocity_iface->clearClaims();

  // Changes state to INITIALIZED
  state_ = INITIALIZED;

  return true;
}

/*std::string tiago_controller::JointController::getHardwareInterfaceType() const {
  ROS_ERROR("trying to get hardware interface type tiago_controller");
  return "tiago_controller";
}*/

void tiago_controller::JointController::update(
  const ros::Time& time, const ros::Duration& period) {

  for (int i = 0; i < jNames_.size(); ++i) {
    if (std::isnan(mapQdesired_[jNames_[i]])) {
      ROS_ERROR_STREAM("Desired joint position is NaN for " << jNames_[i]);
    } else {
      joints_[i].setCommand(mapQdesired_[jNames_[i]]);
    }
  }
}

void tiago_controller::JointController::updateCommand(
  const robot_framework_msgs::command::ConstPtr& msg) {
  auto this_t = std::chrono::high_resolution_clock::now();
  auto t_diff =
      std::chrono::duration_cast<std::chrono::microseconds>(this_t - last_t_)
          .count();
  last_t_ = this_t;
  double t_diff_sec = static_cast<double>(t_diff) / 1000000.0;
  ROS_INFO_STREAM("updated command from ROS time loop : " << t_diff_sec);
  for (int i = 0; i < msg->jnames.size(); ++i) {
    mapQdesired_[msg->jnames[i]] = msg->jpositions[i];
  }
}

void tiago_controller::JointController::starting(const ros::Time& time) {
  ROS_INFO("Starting controller tiago_controller");
}

void tiago_controller::JointController::stopping(const ros::Time& time) {
  ROS_INFO("Stopping controller tiago_controller");
}
