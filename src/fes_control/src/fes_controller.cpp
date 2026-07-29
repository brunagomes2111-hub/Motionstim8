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

    reference = 0.0;
    mesurement = 0.0;

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

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();


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
                        reference = msg->position[i];

                        RCLCPP_INFO(get_node()->get_logger(),"Reference %s: %.3f", joint_name_.c_str(), reference);

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
                        mesurement = msg->position[i];
                        break;
                    }
                }
            });

        }else if (control_mode_ == "torque"){

            reference_torque_sub_ =get_node()->create_subscription<sensor_msgs::msg::JointState>("/reference_torque", 10,[this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    if (msg->name[i] == joint_name_)
                    {
                        reference = msg->effort[i];
                        break;
                    }
                }
            });

            torque_sub_ =get_node()->create_subscription<sensor_msgs::msg::JointState>("/joint_torques",10,[this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    if (msg->name[i] == joint_name_)
                    {
                        mesurement = msg->effort[i];
                        break;
                    }
                }
            });

        }

        log_directory_ = msg->log_directory;
        
        if (!log_initialized_)
        {
            std::string path =log_directory_ + "/" + joint_name_ + "_log.csv";

            log_file_.open(path);

            log_file_ << "time,measurement,reference,error,pid\n";

            log_initialized_ = true;
        }

        //inicializar PID e configurar anti-windup
        control_toolbox::AntiWindupStrategy antiwindup;

        //enquanto o comando estiver dentro dos limites, a integral é atualizada, caso contrário a integral não é atualizada
        antiwindup.type = control_toolbox::AntiWindupStrategy::CONDITIONAL_INTEGRATION;

        antiwindup.i_max = 1.0;
        antiwindup.i_min = -1.0;

        pid_.initialize(kp_, ki_, kd_, 1.0, -1.0, antiwindup);

        pid_initialized_ = true;

        RCLCPP_INFO(get_node()->get_logger(),"Control mode: %s | Kp: %.3f | Ki: %.3f | Kd: %.3f",control_mode_.c_str(),kp_, ki_, kd_);
    
    });

   
   
    RCLCPP_INFO(get_node()->get_logger(),"Configured controller for %s ",joint_name_.c_str());

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

    double error = 0.0;
    double dt = period.seconds();

    //calculo do erro e do comando PID 
    error = reference - mesurement;
    command_ = pid_.compute_command(error, dt);

    // Limitar entre -1 e 1
    command_ = std::clamp(command_, -1.0, 1.0);

    if(!command_interfaces_[0].set_value(command_))
    {
        RCLCPP_WARN(get_node()->get_logger(),"Falha a escrever stim_command de %s",joint_name_.c_str());
    }

    double mesurement_data = 0.0;
    double reference_data = 0.0;

    // Log the data to the CSV file
    mesurement_data = mesurement;
    reference_data = reference;
  
    log_file_<< get_node()->now().seconds() << ","<< mesurement_data << ","<< reference_data << ","<< error << ","<< command_<< "\n";

    if(control_mode_ == "position")
    {
        RCLCPP_INFO(get_node()->get_logger(),"Joint: %s | Pos: %.3f | Ref: %.3f | Err: %.3f | PID: %.3f",
            joint_name_.c_str(),
            mesurement,
            reference,
            error,
            command_);
    }
    else if(control_mode_ == "torque")
    {
        RCLCPP_INFO(get_node()->get_logger(),"Joint: %s | Torque: %.3f | Ref: %.3f | Err: %.3f | PID: %.3f",
            joint_name_.c_str(),
            mesurement,
            reference,
            error,
            command_);
    }

    return controller_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(fes_control::Controller, controller_interface::ControllerInterface)