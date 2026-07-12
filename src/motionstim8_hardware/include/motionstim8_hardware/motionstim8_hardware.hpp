#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <cmath>

#include "hardware_interface/system_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "hardware_interface/handle.hpp"

#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "motionstim8_driver/motionstim8_driver.hpp"

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

    bool configured_ = false;

    bool simulation_mode_ = false;

    std::vector<std::string> joint_names_;

    struct StimConfig
    {
        double agonist_pw_max;
        double antagonist_pw_max;

        double agonist_current;
        double antagonist_current;

        int agonist_channel;
        int antagonist_channel;
    };

    std::unordered_map<std::string, StimConfig> stim_configs_;

    double main_time_;

    double group_time_;

    double n_factor_;

    std::string serial_port_;

    std::vector<double> command_;

    std::vector<double> position_;


};

}
