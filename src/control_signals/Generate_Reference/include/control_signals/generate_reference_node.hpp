#pragma once

#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"



namespace control_signals
{
class GenerateReferenceNode : public rclcpp::Node
{
public:

    GenerateReferenceNode();


private:

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, float*> trajectories_;
    size_t index_;

    void update_reference();

};
}