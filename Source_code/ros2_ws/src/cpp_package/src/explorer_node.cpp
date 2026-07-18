#include "cpp_package/frontier_based_exploration.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierBasedExploration>());
    rclcpp::shutdown();
    return 0;
}