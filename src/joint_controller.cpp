#include "tiago_controller/joint_controller.hpp"

namespace tiago_controller
{
  bool JointController::init(
      hardware_interface::PositionJointInterface *position_iface,
      hardware_interface::VelocityJointInterface *velocity_iface,
      ros::NodeHandle &root_nh, ros::NodeHandle &control_nh)
  {
    ROS_INFO("LOADING TIAGO JOINT CONTROLLER... (JointController::init)");
    return true;
  }

  void JointController::readParametersROS(ros::NodeHandle& controller_nh)
  {
    std::string name = "/talos_controller/yaml_inria_wbc";
    if (!getParameter(name, yaml_inria_wbc_, controller_nh))
      ROS_ERROR_STREAM("/talos_controller/yaml_inria_wbc parameter not found, needed for inria_wbc controller parameters");
    ROS_INFO_STREAM("Using yaml:" << yaml_inria_wbc_);
  }

  void JointController::initInriaWbc()
  {
    YAML::Node runtime_config = IWBC_CHECK(YAML::LoadFile(yaml_inria_wbc_));
    auto behavior_yaml = IWBC_CHECK(runtime_config["iwbc_controller"].as<std::string>());
    auto controller_yaml = IWBC_CHECK(runtime_config["iwbc_controller"].as<std::string>());

    auto controller_config = IWBC_CHECK(YAML::LoadFile(controller_yaml));
    auto controller_base_path = controller_config["CONTROLLER"]["base_path"].as<std::string>();
    auto controller_robot_urdf = controller_config["CONTROLLER"]["urdf"].as<std::string>();
    controller_config["CONTROLLER"]["urdf"] = controller_base_path + "/" + controller_robot_urdf;

    ROS_INFO_STREAM("LOADING URDF MODEL FROM: " << controller_config["CONTROLLER"]["urdf"].as<std::string>());

    auto controller_name = IWBC_CHECK(controller_config["CONTROLLER"]["name"].as<std::string>());
    auto base_controller = inria_wbc::controllers::Factory::instance().create(controller_name, controller_config);
    controller_ = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(base_controller);

    auto behavior_config = IWBC_CHECK(YAML::LoadFile(behavior_yaml));
    auto behavior_name = behavior_config["BEHAVIOR"]["name"].as<std::string>();
    behavior_ = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller_, behavior_config);
    ROS_INFO_STREAM("Loaded behavior from factory " << behavior_name);

    wbc_joint_names_ = controller_->controllable_dofs();
    position_cmd_.resize(wbc_joint_names_.size());
    position_cmd_.setZero();
  }

  bool JointController::initRequest(hardware_interface::RobotHW *robot_hw,
                                    ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh, ClaimedResources &claimed_resources)
  {
    ROS_INFO("INIT REQUEST");

    hardware_interface::PositionJointInterface *position_iface =
        robot_hw->get<hardware_interface::PositionJointInterface>();

    if (!position_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type PositionJointInterface."
                " Make sure this is registered in the hardware_interface::RobotHW class.");
      return false;
    }
    ROS_INFO("INIT REQUEST 2");

    position_iface->clearClaims();

    hardware_interface::VelocityJointInterface *velocity_iface =
        robot_hw->get<hardware_interface::VelocityJointInterface>();

    if (!velocity_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type VelocityJointInterface."
                " Make sure this is registered in the hardware_interface::RobotHW class.");
      return false;
    }
    ROS_INFO("INIT REQUEST 3");

    velocity_iface->clearClaims();

    if (!init(position_iface, velocity_iface, root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }
    ROS_INFO("INIT REQUEST 4");

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

    ROS_INFO("INIT request OK");
    return true;
  }

  /*std::string JointController::getHardwareInterfaceType() const {
  ROS_ERROR("trying to get hardware interface type tiago_controller");
  return "tiago_controller";
}*/

  void JointController::update(const ros::Time &time, const ros::Duration &period)
  {
    ROS_INFO("Update");
    std::cout << " update, duration=" << period << std::endl;

    for (int i = 0; i < jNames_.size(); ++i)
    {
      if (std::isnan(mapQdesired_[jNames_[i]]))
      {
        ROS_ERROR_STREAM("Desired joint position is NaN for " << jNames_[i]);
      }
      else
      {
        joints_[i].setCommand(mapQdesired_[jNames_[i]]);
      }
    }
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