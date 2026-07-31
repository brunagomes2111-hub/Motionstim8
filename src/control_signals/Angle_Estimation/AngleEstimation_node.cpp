#include "control_signals/AngleEstimation_node.hpp"

#include <cmath>

namespace control_signals
{

AngleEstimationNode::AngleEstimationNode(): Node("angle_estimation_node")
{

    publisher_ =create_publisher<sensor_msgs::msg::JointState>("/joint_position",10);

    //usada para gerar a função sinusoidal
    timer_ = create_wall_timer(std::chrono::milliseconds(50),std::bind(&AngleEstimationNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Angle estimation started.");
}


void AngleEstimationNode::timer_callback()
{
    sensor_msgs::msg::JointState msg;

    // Incrementa o tempo usado para gerar a função sinusoidal
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
