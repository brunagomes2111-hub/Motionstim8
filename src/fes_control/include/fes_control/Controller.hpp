#pragma once

#include <string>

#include <fstream>
#include <filesystem>

#include "controller_interface/controller_interface.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "control_toolbox/pid.hpp"

#include "fes_bringup/msg/configuration.hpp"

namespace fes_control
{

class Controller: public controller_interface::ControllerInterface
{
public:

    Controller();

    controller_interface::CallbackReturn
    on_init() override;

    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    controller_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type
    update(const rclcpp::Time & time,const rclcpp::Duration & period) override;

private:

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_sub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;

    rclcpp::Subscription<fes_bringup::msg::Configuration>::SharedPtr configuration_sub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_torque_sub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr torque_sub_;

    bool log_initialized_ = false;
    std::string log_directory_;

    std::string control_mode_ = "";
    std::string joint_name_ = "";

    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;

    double reference = 0.0;
    double mesurement = 0.0;

    double command_ = 0.0;

    bool pid_initialized_ = false;

    control_toolbox::Pid pid_;

    std::ofstream log_file_;
};

}
