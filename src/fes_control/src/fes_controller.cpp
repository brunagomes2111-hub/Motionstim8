#include "fes_control/Controller.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <fstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "control_toolbox/pid.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/subscription.hpp"

namespace fes_control
{

Controller::Controller(): controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn
Controller::on_init()
{
    joint_name_ = auto_declare<std::string>("joint", "");

    // Valor de saida do PID que corresponde a estimulacao maxima (saturacao).
    // E o ganho de normalizacao: PID/output_scale, limitado a [-1, 1].
    auto_declare<double>("output_scale", 10.0);

    desired_position_ = 0.0;

    current_position_ = 0.0;

    command_ = 0.0;


    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Controller::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;

    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.push_back(joint_name_ + "/stim_command");

    return config;
}

controller_interface::InterfaceConfiguration
Controller::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;

    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.push_back(joint_name_ + "/position");

    return config;
}

controller_interface::CallbackReturn
Controller::on_configure(const rclcpp_lifecycle::State &)
{
    joint_name_ = get_node()->get_parameter("joint").as_string();

    output_scale_ = get_node()->get_parameter("output_scale").as_double();

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();

    if(output_scale_ <= 0.0)
    {
        RCLCPP_WARN(get_node()->get_logger(),"output_scale invalido (%.3f), a usar 1.0", output_scale_);
        output_scale_ = 1.0;
    }


    configuration_sub_ =get_node()->create_subscription<fes_bringup::msg::Configuration>("/configuration", qos,[this](const fes_bringup::msg::Configuration::SharedPtr msg)
    {
        control_mode_ = msg->control_mode;

        kp_ = msg->kp;
        ki_ = msg->ki;
        kd_ = msg->kd;

        if(control_mode_ == "position"){

            reference_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>("/reference_joint_states", 10,[this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                for(size_t i = 0; i < msg->name.size(); i++)
                {
                    if(msg->name[i] == joint_name_)
                    {
                        desired_position_ = msg->position[i];

                        RCLCPP_INFO(get_node()->get_logger(),"Reference %s: %.3f", joint_name_.c_str(), desired_position_);

                        break;
                    }
                }
            });

            position_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>("/joint_position", 10,[this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                for(size_t i = 0; i < msg->name.size(); i++)
                {
                    if(msg->name[i] == joint_name_)
                    {
                        current_position_ = msg->position[i];
                        break;
                    }
                }
            });

        }else if (control_mode_ == "torque"){

            //criar dois subscribers do torque

        }

        log_directory_ = msg->log_directory;
        
        if (!log_initialized_)
        {
            std::string path =log_directory_ + "/" + joint_name_ + "_log.csv";

            log_file_.open(path);

            log_file_ << "time,position,reference,error,pid,norm\n";

            log_initialized_ = true;
        }

        control_toolbox::AntiWindupStrategy antiwindup;

        antiwindup.type = control_toolbox::AntiWindupStrategy::CONDITIONAL_INTEGRATION;

        antiwindup.i_max = output_scale_;
        antiwindup.i_min = -output_scale_;

        pid_.initialize(kp_,ki_,kd_,output_scale_,-output_scale_,antiwindup);

        pid_initialized_ = true;

        RCLCPP_INFO(get_node()->get_logger(),"Control mode: %s | Kp: %.3f | Ki: %.3f | Kd: %.3f",control_mode_.c_str(),kp_, ki_, kd_);
    
    });

   
   
    RCLCPP_INFO(get_node()->get_logger(),"Configured controller for %s (output_scale=%.3f)",joint_name_.c_str(), output_scale_);

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Controller::on_activate(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Controller::on_deactivate(const rclcpp_lifecycle::State &)
{
    for(auto & interface : command_interfaces_)
    {
        (void) interface.set_value(0.0);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
Controller::update(const rclcpp::Time &, const rclcpp::Duration & period)
{
    
    if (!pid_initialized_)
    {
        return controller_interface::return_type::OK;
    }

    double error = desired_position_ - current_position_;

    double dt = period.seconds();

    if(control_mode_ == "position")
    {
        command_ = pid_.compute_command(error, dt);
    }
    else if(control_mode_ == "torque")
    {
        // Implementação futura
        command_ = 0.0;
    }
    else
    {
        command_ = 0.0;
    }

    // Normaliza para [-1, 1]. O hardware escala para a largura de impulso (PW)
    // de cada junta: positivo = agonista, negativo = antagonista.
    double normalized = std::clamp(command_ / output_scale_, -1.0, 1.0);

    if(!command_interfaces_[0].set_value(normalized))
    {
        RCLCPP_WARN(get_node()->get_logger(),
            "Falha a escrever stim_command de %s", joint_name_.c_str());
    }

    log_file_
    << get_node()->now().seconds() << ","
    << current_position_ << ","
    << desired_position_ << ","
    << error << ","
    << command_ << ","
    << normalized
    << "\n";

    RCLCPP_INFO(get_node()->get_logger(),
        "Joint: %s | Pos: %.3f | Ref: %.3f | Err: %.3f | PID: %.3f | Norm: %.3f",
        joint_name_.c_str(), current_position_, desired_position_, error, command_, normalized);

    return controller_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(fes_control::Controller, controller_interface::ControllerInterface)