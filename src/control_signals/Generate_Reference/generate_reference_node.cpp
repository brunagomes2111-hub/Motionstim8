#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_signals/generate_reference_node.hpp"
#include "control_signals/trajectories.hpp"

#include <fstream>
#include <thread>
#include <cmath>


namespace control_signals
{

GenerateReferenceNode::GenerateReferenceNode(): Node("generate_reference_node"), index_(0)
{

    RCLCPP_INFO(get_logger(), "GenerateReferenceNode started");

    //Cria um publisher para as trajetórias de referência
    reference_pub_ =create_publisher<sensor_msgs::msg::JointState>("/reference_joint_states",10);
    
    //Atribuiçao da junta á sua respetiva trajetória
    trajectories_["knee_left"]  = WRF::Healthy_LK;
    trajectories_["ankle_left"] = WRF::Healthy_LA;

    trajectories_["knee_right"]  = WRF::Healthy_RK;
    trajectories_["ankle_right"] = WRF::Healthy_RA;

    timer_ = create_wall_timer(std::chrono::milliseconds(10),std::bind(&GenerateReferenceNode::update_reference, this));

    RCLCPP_INFO(get_logger(), "Reference generation started.");
   
}


void GenerateReferenceNode::update_reference()
{
    //criada nova msg do tipo JointState usando apenas name e position
    sensor_msgs::msg::JointState msg; 

    msg.name = {
        "knee_left",
        "ankle_left",
        "knee_right",
        "ankle_right"
    };

    msg.position = {
        trajectories_["knee_left"][index_] * M_PI / 180.0,
        trajectories_["ankle_left"][index_] * M_PI / 180.0,
        trajectories_["knee_right"][index_] * M_PI / 180.0,
        trajectories_["ankle_right"][index_] * M_PI / 180.0
    };

    
    reference_pub_->publish(msg); 

    index_++; //avança para o próximo ponto da trajetória

    if(index_ >= TrajectoryLength)
    {
        index_ = 0;
    }
}

}
