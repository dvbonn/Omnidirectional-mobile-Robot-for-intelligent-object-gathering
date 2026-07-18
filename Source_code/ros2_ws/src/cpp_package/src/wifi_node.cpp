#include "cpp_package/socket_handler.h"

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SocketHandler>());
	rclcpp::shutdown();
	return 0;
}
