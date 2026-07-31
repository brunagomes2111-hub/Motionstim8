#include "motionstim8_hardware/motionstim8_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
        RCLCPP_WARN(rclcpp::get_logger("MotionStim8Hardware"),"SIMULATION MODE ativo: a porta serie NAO sera aberta e nada sera enviado ao estimulador");
    }

    //Percorre as joints definidas no URDF e extrai os parâmetros necessários para cada uma
    for(const auto & joint : info.joints)
    {
        if(joint.parameters.empty())// Ignora joints sem parâmetros de estimulação 
        {
            continue;
        }

        joint_names_.push_back(joint.name);

        // Cria a configuração de estimulação da joint a partir dos parâmetros do URDF
        StimConfig config;

        config.extension_pw_max =std::stod(joint.parameters.at("extension_pw_max"));

        config.flexion_pw_max =std::stod(joint.parameters.at("flexion_pw_max"));

        config.extension_pw =std::stod(joint.parameters.at("extension_pw"));

        config.flexion_pw =std::stod(joint.parameters.at("flexion_pw"));

        config.extension_pa_max =std::stod(joint.parameters.at("extension_pa_max"));

        config.flexion_pa_max =std::stod(joint.parameters.at("flexion_pa_max"));

        config.extension_pa =std::stod(joint.parameters.at("extension_pa"));

        config.flexion_pa =std::stod(joint.parameters.at("flexion_pa"));

        config.extension_m_pw_cc =std::stod(joint.parameters.at("extension_m_pw_cc"));

        config.extension_b_pw_cc =std::stod(joint.parameters.at("extension_b_pw_cc"));

        config.flexion_m_pw_cc =std::stod(joint.parameters.at("flexion_m_pw_cc"));

        config.flexion_b_pw_cc =std::stod(joint.parameters.at("flexion_b_pw_cc"));

        config.extension_m_pa_cc =std::stod(joint.parameters.at("extension_m_pa_cc"));

        config.extension_b_pa_cc =std::stod(joint.parameters.at("extension_b_pa_cc"));

        config.flexion_m_pa_cc =std::stod(joint.parameters.at("flexion_m_pa_cc"));

        config.flexion_b_pa_cc =std::stod(joint.parameters.at("flexion_b_pa_cc"));

        config.extension_channel =std::stoi(joint.parameters.at("extension_channel"));

        config.flexion_channel =std::stoi(joint.parameters.at("flexion_channel"));

        //Associa a configuração à joint correspondente
        stim_configs_[joint.name] = config;

        RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"), "Joint %s -> AG:%d ANT:%d", joint.name.c_str(), config.extension_channel, config.flexion_channel);
    }

    command_.resize(joint_names_.size(), 0.0);

    position_.resize(joint_names_.size(), 0.0);

    joint_position_.resize(joint_names_.size(), 0.0);

    return CallbackReturn::SUCCESS;
}

CallbackReturn
MotionStim8Hardware::on_configure(const rclcpp_lifecycle::State &)
{

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    
    

    //subscription para receber a configuração de coativação (enabled/disabled ) do nó de configuração
    configuration_sub_ =get_node()->create_subscription<fes_bringup::msg::Configuration>("/configuration",qos,[this](const fes_bringup::msg::Configuration::SharedPtr msg)
    {
        coactivation_enabled_ = msg->coactivation_enabled;
        control_mode_ = msg->control_mode;
        modulation_mode_ = msg->modulation_mode;

        // Guarda o diretório de log para criar o ficheiro de log de estimulação
        log_directory_ = msg->log_directory;

        if (!stimulation_log_.is_open())
        {
            std::string filename = log_directory_ + "/stimulation_log.csv";

            stimulation_log_.open(filename);

            stimulation_log_
                << "time,"
                << "joint,"
                << "measurement,"
                << "pid,"
                << "modulation_mode,"
                << "coactivation,"
                << "extension_pw,"
                << "flexion_pw,"
                << "extension_pa,"
                << "flexion_pa"
                << std::endl;

            RCLCPP_INFO(get_logger(),"Logging stimulation to %s",filename.c_str());
        }

        RCLCPP_INFO(get_logger(),"Control mode: %s | Coactivation: %s | Modulation: %s",control_mode_.c_str(),coactivation_enabled_ ? "enabled" : "disabled",modulation_mode_.c_str());    
    });

    position_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>("/joint_position",10,[this](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for(size_t i = 0; i < msg->name.size(); i++)
        {
            auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);

            if(it != joint_names_.end())
            {
                size_t index = std::distance(joint_names_.begin(), it);

                joint_position_[index] = msg->position[i];
            }
        }
    });



    if(simulation_mode_)
    {
        RCLCPP_WARN(rclcpp::get_logger("MotionStim8Hardware"),"SIMULATION MODE: a saltar abertura da porta serie e inicializacao do estimulador");

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

        channels.push_back(config.extension_channel);
        channels.push_back(config.flexion_channel);
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
        state_interfaces.emplace_back(joint_names_[i],hardware_interface::HW_IF_POSITION,&position_[i]);   
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

    std::vector<double> pulse_amplitude;

    std::vector<double> mode;

    //Para cada joint vai buscar os valores config, PW máximo, corrente, canais e o comando PID
    for(size_t i = 0; i < joint_names_.size(); i++)
    {
        const auto & config = stim_configs_.at(joint_names_[i]);

        double output_pid = std::clamp(command_[i], -1.0, 1.0);

        //PW
        double extension_pw = 0.0;
        double flexion_pw = 0.0;
        double extension_pw_pid = 0.0;
        double flexion_pw_pid = 0.0;

        //PA
        double extension_pa = 0.0;
        double flexion_pa = 0.0;
        double extension_pa_pid = 0.0;
        double flexion_pa_pid = 0.0;

      

        //--------------------------- PW MODULATION ------------------------------//

        

        if(modulation_mode_ == "pw_modulation")
        {
            RCLCPP_DEBUG(get_logger(),"Modulation mode PW implemented.");
       
            // Coativação: calcula a PW do canal oposto com base na posição da articulação e nos parâmetros de coativação
            if (control_mode_ == "position")
            {
                coactivation_Extension =config.extension_m_pw_cc * joint_position_[i] + config.extension_b_pw_cc;

                coactivation_Flexion = config.flexion_m_pw_cc * joint_position_[i] + config.flexion_b_pw_cc;

            }
            else if (control_mode_ == "torque")
            {
                // cálculo da coativação para o modo torque
                coactivation_Extension = 0.0;
                coactivation_Flexion = 0.0;
            }

            // Se o comando for positivo, ativa o canal extension se negativo ativa o canal flexion
            // Se a coativação estiver desativada, o canal oposto recebe PW=0

            if (output_pid > 0.0)
            {
                
                // Converte o comando PID normalizado [0,1] em Pulse Width (µs)
                extension_pw_pid = output_pid * config.extension_pw_max;
            
                // Coativação: ativa também o flexion com a PW configurada
                if (coactivation_enabled_)
                {
                    
                    extension_pw = weight_PID * extension_pw_pid + weight_CC * coactivation_Extension;
                    flexion_pw = weight_CC * coactivation_Flexion;
                
                }else{

                    extension_pw = extension_pw_pid;
                    flexion_pw = 0.0;

                }
            }
            else if (output_pid < 0.0)
            {
            
                double pid_abs = std::abs(output_pid);

                // Converte o comando PID normalizado [0,1] em Pulse Width (µs)
                flexion_pw_pid = pid_abs * config.flexion_pw_max;

                // Coativação: ativa também o extension com a PW configurada
                if (coactivation_enabled_)
                {
                    flexion_pw = weight_PID * flexion_pw_pid + weight_CC * coactivation_Flexion;
                    extension_pw = weight_CC * coactivation_Extension;

                }else{
                    
                    flexion_pw = flexion_pw_pid;
                    extension_pw = 0.0;
                }
            }
            else
            {
                extension_pw = 0.0;
                flexion_pw = 0.0;
            }

             // Saturação física de cada músculo
            extension_pw = std::clamp(extension_pw,0.0,config.extension_pw_max);

            flexion_pw = std::clamp(flexion_pw,0.0,config.flexion_pw_max);

            pulse_width.push_back(extension_pw);
            pulse_width.push_back(flexion_pw);

            pulse_amplitude.push_back(config.extension_pa);
            pulse_amplitude.push_back(config.flexion_pa);

            


        //----------------------------------- PA MODULATION -------------------------------//

        

        }else if(modulation_mode_ == "pa_modulation"){

            RCLCPP_DEBUG(get_logger(),"Modulation mode PA implemented.");

            if(control_mode_ == "position")
            {
                coactivation_Extension =config.extension_m_pa_cc * joint_position_[i] + config.extension_b_pa_cc;

                coactivation_Flexion = config.flexion_m_pa_cc * joint_position_[i] + config.flexion_b_pa_cc;

            }
            else if (control_mode_ == "torque")
            {
                // cálculo da coativação para o modo torque
                coactivation_Extension = 0.0;
                coactivation_Flexion = 0.0;
            }

            if(output_pid > 0.0)
            {
                extension_pa_pid = output_pid * config.extension_pa_max;
        
                // Coativação: ativa também o flexion com a PW configurada
                if (coactivation_enabled_)
                {
                    
                    extension_pa = weight_PID * extension_pa_pid + weight_CC * coactivation_Extension;
                    flexion_pa = weight_CC * coactivation_Flexion;
                
                }else{

                    extension_pa = extension_pa_pid;
                    flexion_pa = 0.0;

                }
            }
            else if(output_pid < 0.0)
            {
                double pid_abs = std::abs(output_pid);

                // Converte o comando PID normalizado [0,1] em Pulse Width (µs)
                flexion_pa_pid = pid_abs * config.flexion_pa_max;

                // Coativação: ativa também o extension com a PW configurada
                if (coactivation_enabled_)
                {
                    flexion_pa = weight_PID * flexion_pa_pid + weight_CC * coactivation_Flexion;
                    extension_pa = weight_CC * coactivation_Extension;

                }else{
                    
                    flexion_pa = flexion_pa_pid;
                    extension_pa = 0.0;
                }
            }
            else
            {
                extension_pa = 0.0;
                flexion_pa = 0.0;
            }

            // Saturação física de cada músculo
            extension_pa = std::clamp(extension_pa,0.0,config.extension_pa_max);
            flexion_pa = std::clamp(flexion_pa,0.0,config.flexion_pa_max);

            pulse_amplitude.push_back(extension_pa);
            pulse_amplitude.push_back(flexion_pa);

            pulse_width.push_back(config.extension_pw);
            pulse_width.push_back(config.flexion_pw);
        
        }
        else
        {
            RCLCPP_ERROR(get_logger(),"Invalid modulation mode: %s",modulation_mode_.c_str());
        }

        //Modo single pulses
        mode.push_back(0); //canal extension
        mode.push_back(0); //canal flexion

        if(modulation_mode_ == "pw_modulation")
        {
            
            RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"),"Joint: %s | output_pid: %.2f | AG PW: %.2f | ANT PW: %.2f",
                joint_names_[i].c_str(),output_pid,extension_pw, flexion_pw);

        }
        else if(modulation_mode_ == "pa_modulation")
        {

            RCLCPP_INFO(rclcpp::get_logger("MotionStim8Hardware"),"Joint: %s | output_pid: %.2f | AG PA: %.2f | ANT PA: %.2f",
                joint_names_[i].c_str(),output_pid,extension_pa,flexion_pa);

        }

        double t = get_node()->now().seconds();

        stimulation_log_
        << t << ","
        << joint_names_[i] << ","
        << joint_position_[i] << ","
        << output_pid << ","
        << modulation_mode_ << ","
        << (coactivation_enabled_ ? 1 : 0) << ","
        << extension_pw << ","
        << flexion_pw << ","
        << extension_pa << ","
        << flexion_pa
        << std::endl;
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


    if(!driver_.sendUpdate(pulse_width,pulse_amplitude,mode))
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

    if(stimulation_log_.is_open())
    {
        stimulation_log_.close();
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MotionStim8Hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(motionstim8_hardware::MotionStim8Hardware, hardware_interface::SystemInterface)