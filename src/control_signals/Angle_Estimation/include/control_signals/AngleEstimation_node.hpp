#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


namespace control_signals
{

class AngleEstimation_node : public rclcpp::Node
{
public:

    AngleEstimation_node();


private:

    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    publisher_;

    rclcpp::TimerBase::SharedPtr
    timer_;

    double time_;
};

}


