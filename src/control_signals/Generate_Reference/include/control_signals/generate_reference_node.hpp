#pragma once

#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "fes_bringup/msg/configuration.hpp"

namespace control_signals
{

class GenerateReferenceNode : public rclcpp::Node
{
public:

    GenerateReferenceNode();

private:

    void update_reference();

    void configurationCallback(const fes_bringup::msg::Configuration::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_position_pub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_torque_pub_;

    rclcpp::Subscription<fes_bringup::msg::Configuration>::SharedPtr configuration_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, float*> position_trajectories_;
    std::unordered_map<std::string, float*> torque_trajectories_;

    std::string control_mode_;

    bool configured_{false};

    size_t index_;
};

}