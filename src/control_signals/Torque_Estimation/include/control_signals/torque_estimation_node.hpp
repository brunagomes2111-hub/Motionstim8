#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace control_signals
{

class TorqueEstimationNode : public rclcpp::Node
{
public:
    TorqueEstimationNode();

private:
    void updateTorque();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace control_signals