#include "control_signals/AngleEstimation_node.hpp"

#include <cmath>

namespace control_signals
{

AngleEstimation_node::AngleEstimation_node(): Node("angle_estimation_node")
{
    //usada para gerar a função sinusoidal
    time_ = 0.0;

    publisher_ =create_publisher<sensor_msgs::msg::JointState>("/joint_position",10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50),std::bind(&AngleEstimation_node::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Angle estimation started.");
}


void AngleEstimation_node::timer_callback()
{
    sensor_msgs::msg::JointState msg;

    time_ += 0.05;

    msg.name = {
    "hip_left",
    "knee_left",
    "ankle_left",
    "hip_right",
    "knee_right",
    "ankle_right"
    };

    msg.position = {
    0.3 * std::sin(time_),   // hip_left
    0.7 * std::sin(time_),   // knee_left
    0.5 * std::sin(time_),   // ankle_left
    0.3 * std::sin(time_),   // hip_right
    0.7 * std::sin(time_),   // knee_right
    0.5 * std::sin(time_)    // ankle_right
    };

    msg.header.stamp = now();

    publisher_->publish(msg);
}



}
