#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_signals/generate_reference_node.hpp"
#include "control_signals/position_trajectories.hpp"
#include "control_signals/torque_trajectories.hpp"

#include <fstream>
#include <thread>
#include <cmath>


namespace control_signals
{

    GenerateReferenceNode::GenerateReferenceNode(): Node("generate_reference_node"), index_(0)
    {
        rclcpp::QoS qos(1);
        qos.reliable();
        qos.transient_local();

        RCLCPP_INFO(get_logger(), "GenerateReferenceNode started");

        //Cria um publisher para as trajetórias de referência e torque
        reference_position_pub_ =create_publisher<sensor_msgs::msg::JointState>("/reference_joint_states",10);
        
        reference_torque_pub_ =create_publisher<sensor_msgs::msg::JointState>("/reference_torque", 10);
        
        //subscriber da msg de configuração
        configuration_sub_ = create_subscription<fes_bringup::msg::Configuration>("/configuration",qos,std::bind(&GenerateReferenceNode::configurationCallback,this,std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::milliseconds(10),std::bind(&GenerateReferenceNode::update_reference, this));

        RCLCPP_INFO(get_logger(), "Reference generation started.");
    
    }

    void GenerateReferenceNode::configurationCallback(const fes_bringup::msg::Configuration::SharedPtr msg)
    {
        configured_ = true;

        //vai ao subscriber da msg de configuração e pega o modo de controlo
        control_mode_ = msg->control_mode;
        position_trajectories_.clear();
        torque_trajectories_.clear();

        if (control_mode_ == "position")
        {
            position_trajectories_["knee_left"]  = WRF::Healthy_LK;
            position_trajectories_["ankle_left"] = WRF::Healthy_LA;
            position_trajectories_["knee_right"] = WRF::Healthy_RK;
            position_trajectories_["ankle_right"] = WRF::Healthy_RA;
        }
        else if (control_mode_ == "torque")
        {
            torque_trajectories_["ankle_right"] = WRF::Healthy_RA_Torque;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Unknown control mode: %s", control_mode_.c_str());
            return;
        }

        index_ = 0;
    }


    void GenerateReferenceNode::update_reference()
    {
        if (!configured_)
        {
            return;
        }

        // Cria uma mensagem JointState para publicar a referência
        sensor_msgs::msg::JointState msg; 

        msg.name = {
            "knee_left",
            "ankle_left",
            "knee_right",
            "ankle_right"
        };

        if (control_mode_ == "position")
        {
            msg.position = {
            position_trajectories_["knee_left"][index_] * M_PI / 180.0,
            position_trajectories_["ankle_left"][index_] * M_PI / 180.0,
            position_trajectories_["knee_right"][index_] * M_PI / 180.0,
            position_trajectories_["ankle_right"][index_] * M_PI / 180.0
            };

            msg.effort.resize(msg.name.size(), 0.0);

            reference_position_pub_->publish(msg);

        }
        else if (control_mode_ == "torque")
        {
            msg.position.resize(msg.name.size(), 0.0);

            msg.effort = {
                0.0,
                0.0,
                0.0,
                torque_trajectories_["ankle_right"][index_]
            };

            reference_torque_pub_->publish(msg);
        }
    

        index_++; //avança para o próximo ponto da trajetória

        if (control_mode_ == "position")
        {
            if (index_ >= PositionTrajectoryLength)

                index_ = 0;
        }
        else
        {
            if (index_ >= TorqueTrajectoryLength)

                index_ = 0;
        }
            }

}
