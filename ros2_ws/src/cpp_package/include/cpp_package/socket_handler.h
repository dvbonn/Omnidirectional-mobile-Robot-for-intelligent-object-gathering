#ifndef SOCKET_HANDLER_H
#define SOCKET_HANDLER_H

#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

// #include "wifi_handler.h"
#include "message_type.h"

#include "cpp_package/mdns_service.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using namespace std::chrono_literals;

class SocketHandler : public rclcpp::Node
{
    private:
        int listen_sock;
        bool is_running;
        std::thread socket_thread;
        std::vector<std::thread> client_threads;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        state_t robot_state;
        std::vector<int> client_sockets;

        void receive_task(int sock);
        void send_task(int sock);
        void socket_thread_task();
        void timer_callback();
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    public:
        SocketHandler() : Node("socket_handler")
        {
            // WifiHandler::getInstance()->startAP();
            MDNSService::getInstance();
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            timer_ = this->create_wall_timer(20ms, std::bind(&SocketHandler::timer_callback, this));
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SocketHandler::cmd_vel_callback, this, std::placeholders::_1));

            is_running = true;
            socket_thread = std::thread(&SocketHandler::socket_thread_task, this);
        };
        ~SocketHandler()
        {
            // WifiHandler::getInstance()->stopAP();
            is_running = false;
            if (socket_thread.joinable())
            {
                socket_thread.join();
            }
            for (auto& thread : client_threads)
            {
                if (thread.joinable())
                {
                    thread.join();
                }
            }
        }
};


#endif