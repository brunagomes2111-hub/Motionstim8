#include "rclcpp/rclcpp.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

#include "control_signals/generate_reference_node.hpp"
#include "control_signals/AngleEstimation_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto generate_reference_node = std::make_shared<control_signals::GenerateReferenceNode>();

    auto angle_estimation_node =std::make_shared<control_signals::AngleEstimation_node>();

    rclcpp::experimental::executors::EventsExecutor executor;

    executor.add_node(generate_reference_node);
    executor.add_node(angle_estimation_node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}