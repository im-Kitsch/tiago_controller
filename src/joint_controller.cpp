#include <boost/filesystem.hpp>
#include "tiago_controller/joint_controller.hpp"
#include <inria_wbc/utils/trajectory_handler.hpp>

namespace tiago_controller
{
  bool JointController::init(
      hardware_interface::PositionJointInterface *position_iface,
      hardware_interface::VelocityJointInterface *velocity_iface,
      hardware_interface::JointStateInterface *joint_state_iface,
      ros::NodeHandle &root_nh, ros::NodeHandle &control_nh)
  {
    ROS_INFO("LOADING TIAGO JOINT CONTROLLER... (JointController::init)");

    readParametersROS(control_nh); // we need this to get the iwbc.yaml
    initInriaWbc();                // we need to call this first to get the list of joints

    for (auto &jt_name : wbc_joint_names_)
    {
      try
      {
        rc_joints_.push_back(position_iface->getHandle(jt_name));
        rc_joint_states_.push_back(joint_state_iface->getHandle(jt_name));
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find joint handle '" << jt_name << " in initJoints");
        stop_controller_ = false;
        return false;
      }
    }
    ROS_INFO_STREAM("initJoints: done [" << wbc_joint_names_.size() << "  joints]");

    stop_controller_ = false;
    return true;
  }

  void JointController::readParametersROS(ros::NodeHandle &controller_nh)
  {
    {
      std::string name = "/tiago_controller/yaml_inria_wbc";
      if (!getParameter(name, yaml_inria_wbc_, controller_nh))
        ROS_ERROR_STREAM("/talos_controller/yaml_inria_wbc parameter not found, needed for inria_wbc controller parameters");
      ROS_INFO_STREAM("Using yaml:" << yaml_inria_wbc_);
    }
    {
      std::string name = "/tiago_controller/base_directory";
      if (!getParameter(name, base_directory_, controller_nh))
        ROS_ERROR_STREAM("/talos_controller/base_directory parameter not found, needed for inria_wbc controller parameters");
      ROS_INFO_STREAM("Using base directory:" << base_directory_);
    }
  }

  // initialize the whole body controller from the yaml yaml_inria_wbc_ (from ros params)
  void JointController::initInriaWbc()
  {
    ROS_INFO_STREAM("main YAML file:" << base_directory_ + "/" + yaml_inria_wbc_);
    try
    {
      YAML::Node runtime_config = IWBC_CHECK(YAML::LoadFile(base_directory_ + "/" + yaml_inria_wbc_));
      auto behavior_yaml = IWBC_CHECK(runtime_config["iwbc_behavior"].as<std::string>());
      auto controller_yaml = IWBC_CHECK(runtime_config["iwbc_controller"].as<std::string>());
      ROS_INFO_STREAM("Using controller yaml:" << base_directory_ + "/" + controller_yaml);
      ROS_INFO_STREAM("Using behavior yaml:" << base_directory_ + "/" + behavior_yaml);

      auto controller_config = IWBC_CHECK(YAML::LoadFile(base_directory_ + "/" + controller_yaml));
      auto controller_base_path = controller_config["CONTROLLER"]["base_path"].as<std::string>();
      auto controller_robot_urdf = controller_config["CONTROLLER"]["urdf"].as<std::string>();
      controller_config["CONTROLLER"]["urdf"] = controller_base_path + "/" + controller_robot_urdf;

      ROS_INFO_STREAM("LOADING URDF MODEL FROM: " << controller_config["CONTROLLER"]["urdf"].as<std::string>());

      auto controller_name = IWBC_CHECK(controller_config["CONTROLLER"]["name"].as<std::string>());
      auto base_controller = inria_wbc::controllers::Factory::instance().create(controller_name, controller_config);
      controller_ = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(base_controller);

      auto behavior_config = IWBC_CHECK(YAML::LoadFile(base_directory_ + "/" + behavior_yaml));
      auto behavior_name = behavior_config["BEHAVIOR"]["name"].as<std::string>();
      behavior_ = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller_, behavior_config);
      ROS_INFO_STREAM("Loaded behavior from factory " << behavior_name);

      wbc_joint_names_ = controller_->controllable_dofs();
      init_sequence_duration_ = IWBC_CHECK(runtime_config["init_duration"].as<double>());

    }
    catch (std::exception &e)
    {
      ROS_ERROR_STREAM("TiagoController: Cannot init inria_wbc!" << e.what());
    }
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

    position_iface->clearClaims();

    hardware_interface::VelocityJointInterface *velocity_iface =
        robot_hw->get<hardware_interface::VelocityJointInterface>();

    if (!velocity_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type VelocityJointInterface."
                " Make sure this is registered in the hardware_interface::RobotHW class.");
      return false;
    }
    velocity_iface->clearClaims();

    hardware_interface::JointStateInterface *joint_state_iface = robot_hw->get<hardware_interface::JointStateInterface>();
    if (!joint_state_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s' ."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                hardware_interface::internal::demangledTypeName<hardware_interface::JointStateInterface>().c_str());
      return false;
    }

    if (!init(position_iface, velocity_iface, joint_state_iface, root_nh, controller_nh))
    {
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

    claimed_resources.push_back(hardware_interface::InterfaceResources(
        hardware_interface::internal::demangledTypeName<hardware_interface::JointStateInterface>(),
        joint_state_iface->getClaims()));
    joint_state_iface->clearClaims();

    // Changes state to INITIALIZED
    state_ = INITIALIZED;

    ROS_INFO("INIT request OK");
    return true;
  }

  void JointController::update(const ros::Time &time, const ros::Duration &period)
  {
    if (!init_sequence_q_.empty())
    {
        for (int i = 0; i < wbc_joint_names_.size(); ++i)
          rc_joints_[i].setCommand(init_sequence_q_.front()[i]);
        init_sequence_q_.pop_front();//slow but we don' t care here..
    }
    else
    { // normal QP solver
      try
      {
        behavior_->update(); //could have sensors here
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM(e.what());
        stop_controller_ = true;
        return;
      }

      // send the commands
      if (!stop_controller_)
      {
        auto q = controller_->q().tail(wbc_joint_names_.size());
        //std::cout<<"q: "<<q.transpose()<<std::endl;
        assert(rc_joint_.size() == q.size());
        for (size_t i = 0; i < wbc_joint_names_.size(); ++i)
          rc_joints_[i].setCommand(q[i]);
      } // we do nothing
      else
      {
        ROS_INFO("Tiago controller is stopped");
      }
    }
  }

  void JointController::starting(const ros::Time &time)
  {
    ROS_INFO("Starting controller tiago_controller");
    initInriaWbc();
    stop_controller_ = false;
    
    // get the current position
    Eigen::VectorXd current_joint_pos = Eigen::VectorXd::Zero(wbc_joint_names_.size());
    for (size_t i = 0; i < wbc_joint_names_.size(); ++i)
      current_joint_pos[i] = rc_joint_states_[i].getAbsolutePosition();
    ROS_INFO_STREAM("tiago_controller starting, joints=" << current_joint_pos.transpose());
    // minimum jerk initialization trajectory for init sequence
    auto q_v = trajectory_handler::compute_traj(
        current_joint_pos, 
        controller_->q0(), 
        controller_->dt(), 
        init_sequence_duration_);
    init_sequence_q_ = std::list<Eigen::VectorXd>(q_v.begin(), q_v.end());
  }

  void JointController::stopping(const ros::Time &time)
  {
    ROS_INFO("Stopping controller tiago_controller");
    stop_controller_ = true;
  }
}