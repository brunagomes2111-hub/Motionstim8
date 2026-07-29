#include "control_signals/torque_estimation_node.hpp"

namespace control_signals
{

    TorqueEstimationNode::TorqueEstimationNode(): Node("torque_estimation_node")
    {
        RCLCPP_INFO(get_logger(), "TorqueEstimationNode started.");

        torque_pub_ =create_publisher<sensor_msgs::msg::JointState>("/joint_torques", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(10),std::bind(&TorqueEstimationNode::updateTorque, this));

    }

    void TorqueEstimationNode::updateTorque()
    {
        sensor_msgs::msg::JointState msg;

        msg.header.stamp = now();

        msg.name = {
            "knee_left",
            "ankle_left",
            "knee_right",
            "ankle_right"
        };

        msg.position.resize(4, 0.0);
        msg.velocity.resize(4, 0.0);

        static double t = 0.0;

        t += 0.01;
        
        msg.effort = {
            0.0,
            0.0,
            0.0,
            10.0 * std::sin(2.0 * M_PI * 0.5 * t)
        };

        torque_pub_->publish(msg);
    }

} // namespace control_signals