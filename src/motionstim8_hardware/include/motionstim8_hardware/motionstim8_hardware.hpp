#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <cmath>
#include <fstream>


#include "hardware_interface/system_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "hardware_interface/handle.hpp"

#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "motionstim8_driver/motionstim8_driver.hpp"

#include "fes_bringup/msg/configuration.hpp"



namespace motionstim8_hardware
{

using CallbackReturn =rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MotionStim8Hardware : public hardware_interface::SystemInterface
{
public:

    CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

    CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface>export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>export_command_interfaces() override;

    hardware_interface::return_type
    read(const rclcpp::Time & time,const rclcpp::Duration & period) override;

    hardware_interface::return_type
    write(const rclcpp::Time & time,const rclcpp::Duration & period) override;

    CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

private:

    // Driver responsável pela comunicação de baixo nível com o MotionStim8
    motionstim8_driver::MotionStim8Driver driver_;

    std::ofstream stimulation_log_;

    bool configured_ = false;

    bool simulation_mode_ = false;

    std::vector<std::string> joint_names_;

    std::vector<double> joint_position_;

    struct StimConfig
    {
        double extension_pw_max;
        double flexion_pw_max;

        double extension_pa;
        double flexion_pa;

        double extension_pa_max;
        double flexion_pa_max;

        double extension_pw;
        double flexion_pw;

        double flexion_b_pw_cc;
        double extension_b_pw_cc;

        double flexion_m_pw_cc;
        double extension_m_pw_cc;

        double flexion_b_pa_cc;
        double extension_b_pa_cc;

        double flexion_m_pa_cc;
        double extension_m_pa_cc;

        int extension_channel;
        int flexion_channel;
    };

    rclcpp::Subscription<fes_bringup::msg::Configuration>::SharedPtr configuration_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;

    std::unordered_map<std::string, StimConfig> stim_configs_;

    double main_time_;

    double group_time_;

    double n_factor_;

    std::string serial_port_;

    std::vector<double> command_;

    std::vector<double> position_;

    bool coactivation_enabled_{false};

    std::string control_mode_;

    std::string modulation_mode_;

    std::string log_directory_;

    double coactivation_Extension = 0.0;
    double coactivation_Flexion = 0.0;

    const double weight_PID = 0.5;
    const double weight_CC = 0.5;

    

};

}
