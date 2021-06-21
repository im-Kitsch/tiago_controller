#ifndef JointController_HPP_
#define JointController_HPP_

#include "std_msgs/Float64MultiArray.h"
#include <robot_framework_msgs/command.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Core>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <boost/bind.hpp>
#include <chrono>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace tiago_controller {
    class JointController : public controller_interface::ControllerBase {
    public:
        JointController() {}
        ~JointController() {}
        void update(const ros::Time& time, const ros::Duration& period);
        void updateCommand(const robot_framework_msgs::command::ConstPtr& msg);

        bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh, ClaimedResources& claimed_resources) override;
        // std::string getHardwareInterfaceType() const override;

        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;

    private:
        bool init(hardware_interface::PositionJointInterface* position_iface,
            hardware_interface::VelocityJointInterface* velocity_iface,
            ros::NodeHandle& root_nh, ros::NodeHandle& control_nh);
        std::vector<hardware_interface::JointHandle> joints_;
        std::vector<std::string> jNames_;
        std::map<std::string, double> mapQdesired_, mapQInit_;
        std::shared_ptr<ros::Subscriber> joint_command_sub_;
        double cont;
        std::chrono::high_resolution_clock::time_point last_t_;
        double dt_ = 0.001;
    };
} // namespace tiago_controller

PLUGINLIB_EXPORT_CLASS(tiago_controller::JointController, controller_interface::ControllerBase)

#endif // JointController_HPP_
