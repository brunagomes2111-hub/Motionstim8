#pragma once

#include "rclcpp/rclcpp.hpp"
#include "fes_bringup/msg/configuration.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

namespace fes_bringup
{

    class ConfigurationNode : public rclcpp::Node
    {
        public:
            ConfigurationNode();

            void configureSystem();

            bool patientExists(const std::string & name);

            void savePatientInfo();

       private:

            std::string getPatientName();
            std::string getSex();

            uint32_t getAge();

            double getHeight();
            double getWeight();

            int getControlModeSelection();

            bool getCoactivationEnabled();

            double getGain(const std::string & name);

            bool activateSelectedControllers();

            rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;

            int getJointSelection();

            int getModulationModeSelection();
            
            std::string CurrentDate();

            void createTrialDirectory();

            int nextTrialNumber(const std::string & date_directory);

            void saveExperimentInfo();

            fes_bringup::msg::Configuration configuration_;
            
            rclcpp::Publisher<fes_bringup::msg::Configuration>::SharedPtr configuration_publisher_;

    };

}