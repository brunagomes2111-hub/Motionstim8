#include "rclcpp/rclcpp.hpp"
#include "fes_bringup/configuration_node.hpp"

#include <iostream>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>

namespace fes_bringup
{
    ConfigurationNode::ConfigurationNode() : Node("configuration_node")
    {   
        rclcpp::QoS qos(1);
        qos.reliable();
        qos.transient_local();

        configuration_publisher_ =create_publisher<fes_bringup::msg::Configuration>("/configuration",qos);

        switch_client_ =create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

        RCLCPP_INFO(get_logger(), "Configuration node started.");
    }

    void ConfigurationNode::configureSystem()
    {
        bool finished_selection = false;

        RCLCPP_INFO(get_logger(),"Patient name:");
        configuration_.patient_name = getPatientName();

        bool new_patient = !patientExists(configuration_.patient_name);

        if(new_patient)
        {
            RCLCPP_INFO(get_logger(),"New patient.");

            RCLCPP_INFO(get_logger(),"Sex (M/F):");
            configuration_.sex = getSex();

            RCLCPP_INFO(get_logger(),"Age:");
            configuration_.age = getAge();

            RCLCPP_INFO(get_logger(),"Height (m):");
            configuration_.height = getHeight();

            RCLCPP_INFO(get_logger(),"Weight (kg):");
            configuration_.weight = getWeight();

            savePatientInfo();
        }
        else
        {
            RCLCPP_INFO(get_logger(),"Existing patient found.");
        }

        // Perguntar ao utilizador
        RCLCPP_INFO(get_logger(), "Available joints:");
        
        RCLCPP_INFO(get_logger(), "1 - knee_left");
        RCLCPP_INFO(get_logger(), "2 - ankle_left");
        RCLCPP_INFO(get_logger(), "3 - knee_right");
        RCLCPP_INFO(get_logger(), "4 - ankle_right");
        RCLCPP_INFO(get_logger(), "5 - all.");
        RCLCPP_INFO(get_logger(), "0 - Finish selection.\n");


        while(!finished_selection)
        {
            int selected_joint = getJointSelection();
        
            switch(selected_joint)
            {
                case 0:
                    finished_selection = true;
                    break;

                case 1:
                    configuration_.joints.push_back("knee_left");
                    break;

                case 2:
                    configuration_.joints.push_back("ankle_left");
                    break;

                case 3:
                    configuration_.joints.push_back("knee_right");
                    break;

                case 4:
                    configuration_.joints.push_back("ankle_right");
                    break;

                case 5:
                    configuration_.joints =
                    {
                        "knee_left",
                        "ankle_left",
                        "knee_right",
                        "ankle_right"
                    };
                    
                    finished_selection = true;
                    
                    break;

                default:
                    //caso haja erro, o nó não chega a arrancar
                    throw std::runtime_error("Invalid option selected");
            }
        }

            RCLCPP_INFO(get_logger(), "Select control mode:");
            RCLCPP_INFO(get_logger(), " 1 - Position");
            RCLCPP_INFO(get_logger(), " 2 - Torque ");
            
            int control_mode = getControlModeSelection();

            switch (control_mode)
            {
            case 1:
                configuration_.control_mode = "position";
                break;

            case 2:
                configuration_.control_mode = "torque";
                break;
            
            default:
                throw std::runtime_error("Invalid control mode selected");
            }

            if (!activateSelectedControllers())
            {
                RCLCPP_ERROR(get_logger(), "Failed to activate controllers.");
                return;
            }

            RCLCPP_INFO(get_logger(), "Enter Kp gain:");
            configuration_.kp = getGain("Kp");
            RCLCPP_INFO(get_logger(), "Enter Ki gain:");
            configuration_.ki = getGain("Ki");
            RCLCPP_INFO(get_logger(), "Enter Kd gain:");
            configuration_.kd = getGain("Kd");

            rclcpp::sleep_for(std::chrono::milliseconds(200));

            createTrialDirectory();

            saveExperimentInfo();

            configuration_publisher_->publish(configuration_);

            RCLCPP_INFO(get_logger(), "Configuration published.");
            
            for (const auto & joint : configuration_.joints)
            {
                RCLCPP_INFO(get_logger(), "Joint: %s", joint.c_str());
            }

            RCLCPP_INFO(get_logger(),"Control mode: %s",configuration_.control_mode.c_str());
   
    }

    bool ConfigurationNode::activateSelectedControllers()
    {
        if (!switch_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(get_logger(),"Service switch_controller unavailable.");

            return false;
        }

        auto request =std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

        for (const auto & joint : configuration_.joints)
        {
            request->activate_controllers.push_back(joint + "_controller");
        }

        request->strictness =controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

        request->activate_asap = true;

        auto future = switch_client_->async_send_request(request);

        auto result = rclcpp::spin_until_future_complete(get_node_base_interface(),future,std::chrono::seconds(5));

        if (result != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(),"Failed to call switch_controller.");

            return false;
        }

        if (!future.get()->ok)
        {
            RCLCPP_ERROR(get_logger(),"%s", future.get()->message.c_str());

            return false;
        }

        for (const auto & joint : configuration_.joints)
        {
            RCLCPP_INFO(get_logger(),"Activated controller: %s_controller",joint.c_str());
        }

        return true;
    }

    std::string ConfigurationNode::getPatientName()
    {
        std::string name;

        while (true)
        {
            std::cout << "Name: ";

            std::getline(std::cin >> std::ws, name);

            if (!name.empty())
            {
                return name;
            }

            RCLCPP_ERROR(get_logger(), "Invalid name.");
        }
    }

    std::string ConfigurationNode::getSex()
    {
        std::string patient_sex;

        while (true)
        {
            std::cout << "Sex (M/F): ";

            std::cin >> patient_sex;

            if (patient_sex == "M" || patient_sex == "m")
            {
                return "Male";
            }

            if (patient_sex == "F" || patient_sex == "f")
            {
                return "Female";
            }

            RCLCPP_ERROR(get_logger(), "Invalid option. Please enter M or F.");
        }
    }

    uint32_t ConfigurationNode::getAge()
    {
        uint32_t patient_age;

        while (true)
        {
            std::cout << "Age: ";

            if (std::cin >> patient_age && patient_age > 0)
            {
                return patient_age;
            }

            std::cin.clear();
            std::cin.ignore(10000, '\n');

            RCLCPP_ERROR(get_logger(), "Invalid age.");
        }
    }

    double ConfigurationNode::getHeight()
    {
        double patient_height;

        while (true)
        {
            std::cout << "Height (m): ";

            if (std::cin >> patient_height && patient_height > 0.0)
            {
                return patient_height;
            }

            std::cin.clear();
            std::cin.ignore(10000, '\n');

            RCLCPP_ERROR(get_logger(), "Invalid height.");
        }
    }

    double ConfigurationNode::getWeight()
    {
        double patient_weight;

        while (true)
        {
            std::cout << "Weight (kg): ";

            if (std::cin >> patient_weight && patient_weight > 0.0)
            {
                return patient_weight;
            }

            std::cin.clear();
            std::cin.ignore(10000, '\n');

            RCLCPP_ERROR(get_logger(), "Invalid weight.");
        }
    }

    int ConfigurationNode :: getJointSelection()
    {

        int option; //joint

        while(true)
        {
            std::cout << "\nSelect option: ";

            if (std::cin >> option && option >= 0 && option <= 5)
            {
                return option;
            }

            std::cin.clear();
            std::cin.ignore(10000, '\n');

            RCLCPP_ERROR(get_logger(), "Invalid option.");
        }
    }

    int ConfigurationNode::getControlModeSelection()
    {
        int option;//control mode

        while (true)
        {
            std::cout << "\nSelect option: ";

            if (std::cin >> option && option >= 1 && option <= 2)
            {
                return option;
            }

            std::cin.clear();
            std::cin.ignore(10000, '\n');

            RCLCPP_ERROR(get_logger(), "Invalid option.");
        }
    }
    
    double ConfigurationNode::getGain(const std::string & name)
    {
        double value;

        while (true)
        {
            std::cout << name << ": ";

            if (std::cin >> value)
            {
                return value;
            }

            std::cin.clear();
            std::cin.ignore(10000, '\n');

            RCLCPP_ERROR(get_logger(), "Invalid value.");
        }
    }

    bool ConfigurationNode::patientExists(const std::string &name)
    {
        return std::filesystem::exists("logs/" + name);
    }

    void ConfigurationNode::savePatientInfo()
    {
        std::filesystem::create_directories("logs/" + configuration_.patient_name);

        std::ofstream file("logs/" + configuration_.patient_name + "/patient_info.txt");

        file << "Patient Information\n";
        file << "-------------------\n\n";

        file << "Name: " << configuration_.patient_name << "\n";
        file << "Sex: " << configuration_.sex << "\n";
        file << "Age: " << configuration_.age << "\n";
        file << "Height: " << configuration_.height << " m\n";
        file << "Weight: " << configuration_.weight << " kg\n";
    }

    std::string ConfigurationNode::currentDate()
    {
        auto now = std::chrono::system_clock::now();

        std::time_t t = std::chrono::system_clock::to_time_t(now);

        std::tm tm = *std::localtime(&t);

        std::ostringstream oss;

        oss << std::put_time(&tm, "%Y-%m-%d");

        return oss.str();
    }

    int ConfigurationNode::nextTrialNumber(const std::string & date_directory)
    {
        int max_trial = 0;

        if(!std::filesystem::exists(date_directory))
        {
            return 1;
        }

        for(const auto & entry : std::filesystem::directory_iterator(date_directory))
        {
            if(entry.is_directory())
            {
                std::string name = entry.path().filename().string();

                if(name.rfind("trial",0) == 0)
                {
                    int number = std::stoi(name.substr(5));

                    max_trial = std::max(max_trial, number);
                }
            }
        }

        return max_trial + 1;
    }

    void ConfigurationNode::createTrialDirectory()
    {
        std::string date =
            currentDate();

        std::string date_directory =
            "logs/" +
            configuration_.patient_name +
            "/" +
            date;

        int trial =
            nextTrialNumber(date_directory);

        configuration_.log_directory =
            date_directory +
            "/trial" +
            std::to_string(trial);

        std::filesystem::create_directories(configuration_.log_directory);
    }

    void ConfigurationNode::saveExperimentInfo()
    {
        std::ofstream file(
            configuration_.log_directory +
            "/experiment_info.txt");

        file << "Control mode: "
            << configuration_.control_mode
            << "\n\n";

        file << "Selected joints\n";

        for(const auto & joint : configuration_.joints)
        {
            file << "- " << joint << "\n";
        }

        file << "\n";

        file << "PID\n";

        file << "Kp: " << configuration_.kp << "\n";
        file << "Ki: " << configuration_.ki << "\n";
        file << "Kd: " << configuration_.kd << "\n";
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<fes_bringup::ConfigurationNode>();
    
    node->configureSystem();

    rclcpp::shutdown();

    return 0;
}