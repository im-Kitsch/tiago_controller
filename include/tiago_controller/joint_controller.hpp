#ifndef TIAGO_JointController_HPP_
#define TIAGO_JointController_HPP_

#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <boost/bind.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>




namespace tiago_controller {
    class JointController : public controller_interface::ControllerBase {
    public:
        JointController() {ROS_INFO("tiago_controller::constructor"); }
        ~JointController() {}
        void update(const ros::Time& time, const ros::Duration& period);

        bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh, ClaimedResources& claimed_resources) override;
        // std::string getHardwareInterfaceType() const override;

        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;

    private:
        bool init(hardware_interface::PositionJointInterface* position_iface,
            hardware_interface::VelocityJointInterface* velocity_iface,
            ros::NodeHandle& root_nh, ros::NodeHandle& control_nh);
    };
} // namespace tiago_controller

PLUGINLIB_EXPORT_CLASS(tiago_controller::JointController, controller_interface::ControllerBase)

#endif // JointController_HPP_
