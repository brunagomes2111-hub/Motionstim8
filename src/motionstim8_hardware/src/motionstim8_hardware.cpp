#include "motionstim8_hardware/motionstim8_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>

namespace motionstim8_hardware
{

CallbackReturn
MotionStim8Hardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    const auto & info = params.hardware_info;

    RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"),"Hardware initialized");

    serial_port_ = info.hardware_parameters.at("serial_port");

    main_time_ = std::stod(info.hardware_parameters.at("main_time"));

    group_time_ = std::stod(info.hardware_parameters.at("group_time"));

    n_factor_ = std::stod(info.hardware_parameters.at("n_factor"));

    // Parametro opcional. Se ausente, fica false (modo real).
    auto it = info.hardware_parameters.find("simulation_mode");
    if(it != info.hardware_parameters.end())
    {
        simulation_mode_ = (it->second == "true" || it->second == "1");
    }

    if(simulation_mode_)
    {
        RCLCPP_WARN(rclcpp::get_logger("MotionStim8Hardware"),
            "SIMULATION MODE ativo: a porta serie NAO sera aberta e nada sera enviado ao estimulador");
    }

    //Percorre as joints definidas no URDF e extrai os parâmetros necessários para cada uma
    for(const auto & joint : info.joints)
    {
        if(joint.parameters.empty())// Ignora joints sem parâmetros de estimulação (neste caso as hips)
        {
            continue;
        }

        joint_names_.push_back(joint.name);

        // Cria a configuração de estimulação da joint a partir dos parâmetros do URDF
        StimConfig config;

        config.agonist_pw_max = std::stod(joint.parameters.at("agonist_pw_max"));

        config.antagonist_pw_max = std::stod(joint.parameters.at("antagonist_pw_max"));

        config.agonist_current = std::stod(joint.parameters.at("agonist_current"));

        config.antagonist_current = std::stod(joint.parameters.at("antagonist_current"));

        config.agonist_channel = std::stoi(joint.parameters.at("agonist_channel"));

        config.antagonist_channel = std::stoi(joint.parameters.at("antagonist_channel"));

        //Associa a configuração à joint correspondente
        stim_configs_[joint.name] = config;

        RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"), "Joint %s -> AG:%d ANT:%d", joint.name.c_str(), config.agonist_channel, config.antagonist_channel);
    }

    command_.resize(joint_names_.size(), 0.0);

    position_.resize(joint_names_.size(), 0.0);

    return CallbackReturn::SUCCESS;
}

CallbackReturn
MotionStim8Hardware::on_configure(const rclcpp_lifecycle::State &)
{
    if(simulation_mode_)
    {
        RCLCPP_WARN(rclcpp::get_logger("MotionStim8Hardware"),
            "SIMULATION MODE: a saltar abertura da porta serie e inicializacao do estimulador");

        configured_ = true;

        return CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"), "Opening serial port");

    if (!driver_.connect())
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotionStim8Hardware"),"Failed to connect to MotionStim8.");
        return CallbackReturn::ERROR;
    }

    std::vector<double> channels;

    for(const auto & joint_name : joint_names_)
    {
        const auto & config = stim_configs_.at(joint_name);

        channels.push_back(config.agonist_channel);
        channels.push_back(config.antagonist_channel);
    }

    RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"),"Initializing stimulator");

    if(!driver_.initialize(channels,main_time_,group_time_,n_factor_))
    {
        
        RCLCPP_ERROR(rclcpp::get_logger("MotionStim8Hardware"),"Failed to initialize MotionStim8");

        driver_.disconnect();

        return CallbackReturn::ERROR;
    }

    configured_ = true;

    RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"), "Stimulator configured");

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotionStim8Hardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for(size_t i = 0; i < joint_names_.size(); i++)
    {
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_[i]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotionStim8Hardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(size_t i = 0; i < joint_names_.size(); i++)
    {
        command_interfaces.emplace_back(joint_names_[i], "stim_command", &command_[i]);
    }

    return command_interfaces;
}

hardware_interface::return_type
MotionStim8Hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if(!configured_)
    {
        return hardware_interface::return_type::ERROR;
    }

    std::vector<double> pulse_width;

    std::vector<double> pulse_current;

    std::vector<double> mode;

    //Para cada joint vai buscar os valores config, PW máximo, corrente, canais e o comando PID
    for(size_t i = 0; i < joint_names_.size(); i++)
    {
        const auto & config = stim_configs_.at(joint_names_[i]);

        // Comando calculado pelo PID para esta joint
        double cmd = command_[i];

        double agonist_pw = 0.0;

        double antagonist_pw = 0.0;

        if(cmd > 0.0)
        {
            agonist_pw = cmd * config.agonist_pw_max;
        }
        else if(cmd < 0.0)
        {
            antagonist_pw = -cmd * config.antagonist_pw_max;
        }

        pulse_width.push_back(agonist_pw);
        pulse_width.push_back(antagonist_pw);

        pulse_current.push_back(config.agonist_current);
        pulse_current.push_back(config.antagonist_current);

        //Modo single pulses
        mode.push_back(0); //canal agonista
        mode.push_back(0); //canal antagonista

        RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"),
            "Joint: %s | CMD: %.2f | AG PW: %.2f | ANT PW: %.2f", joint_names_[i].c_str(),
            cmd, agonist_pw, antagonist_pw);
    }

    unsigned int num_channels = static_cast<unsigned int>(pulse_width.size());

    if(num_channels == 0)
    {
        return hardware_interface::return_type::OK;
    }

    // Em simulacao calcula e mostra os PW, mas nao fala com o estimulador.
    if(simulation_mode_)
    {
        RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"), "[SIM] Packet NOT sent (%u channels)", num_channels);

        return hardware_interface::return_type::OK;
    }

    RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"), "Sending packet");

    if(!driver_.sendUpdate(pulse_width,pulse_current,mode))
    {
        configured_ = false;

        driver_.disconnect();

        return hardware_interface::return_type::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"), "Packet sent with %u channels", num_channels);

    return hardware_interface::return_type::OK;
}
CallbackReturn
MotionStim8Hardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    if(!simulation_mode_)
    {
        driver_.disconnect();
        RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"), "Serial port closed");
    }
    configured_ = false;

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MotionStim8Hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(motionstim8_hardware::MotionStim8Hardware, hardware_interface::SystemInterface)