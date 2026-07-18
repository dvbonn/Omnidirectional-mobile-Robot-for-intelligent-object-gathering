#include "cpp_package/socket_handler.h"

// #define SEND_DATA(sock, frame) \
// {\
//     if(!send_frame(sock, frame)) return;\
// }\

#define DISPOSE_SOCKET(sock) \
{\
    close(sock);\
    return;\
}\

// static bool send_frame(int* sock, const frame_t *frame)
// {
//     if(frame->length + 1 > 512)
//     {
//         RCLCPP_ERROR(this->get_logger(), "Frame too long to send");
//         return false;
//     }

//     int err = send(*sock, frame, frame->length + 1, 0);
//     if (err < 0)
//     {
//         RCLCPP_ERROR(this->get_logger(), "Error occurred during sending: errno %d", errno);
//         close(*sock);
//         return false;
//     }
//     // ESP_LOGI(TAG, "Sent %d bytes. Frame %u, length: %u", err, frame->payload, frame->length);
//     return true;
// }

void SocketHandler::receive_task(int sock) 
{
    int len;
    uint8_t rx_buffer[512];
    while (is_running)
    {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        switch(len)
        {
            case -1:
                RCLCPP_ERROR(this->get_logger(), "Error occurred during receiving: errno %d", errno);
                DISPOSE_SOCKET(sock);
                break;
            case 0:
                RCLCPP_INFO(this->get_logger(), "Connection closed");
                DISPOSE_SOCKET(sock);
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Received %d bytes", len);
                auto message = std_msgs::msg::UInt8MultiArray();
                generic_msg_t* msg = (generic_msg_t*)(rx_buffer + 1); //bypass the length byte
                if(msg->cmd == ROBOT_STATE)
                {
                    robot_state = msg->robot_state_msg.robot_state;
                }
                break;
        }
    }
    client_sockets.erase(std::remove(client_sockets.begin(), client_sockets.end(), sock), client_sockets.end());
}

void SocketHandler::send_task(int sock) 
{
    while (is_running)
    {

        std::this_thread::sleep_for(200ms);
    }
}

void SocketHandler::socket_thread_task() 
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if(listen_sock < 0)
    {
            RCLCPP_ERROR(this->get_logger(), "Unable to create socket: errno %d", errno);
            return;          
    }
    uint8_t opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(uint8_t));

    struct sockaddr_storage dest_addr;
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_port = htons(2004);

    if(bind(listen_sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Socket unable to bind: errno %d", errno);
        DISPOSE_SOCKET(listen_sock);
    }

    if(listen(listen_sock, 5) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error occurred during listen: errno %d", errno);
        DISPOSE_SOCKET(listen_sock);          
    }

    uint8_t keepAlive = 1;
    uint8_t keepIdle = 29;
    uint8_t keepInterval = 1;
    uint8_t keepCount = 1;

    RCLCPP_INFO(this->get_logger(), "Socket created, listening on port %d", 2004);

    while (is_running)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for connection...");

        int sock = accept(listen_sock, NULL, NULL);
        if (sock < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to accept connection: errno %d", errno);
            break;
        }

        RCLCPP_INFO(this->get_logger(), "New connection accepted");

        // Set TCP options
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(uint8_t));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(uint8_t));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(uint8_t));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(uint8_t));
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(uint8_t));

        client_sockets.push_back(sock);
        client_threads.push_back(std::thread(&SocketHandler::receive_task, this, sock));
        client_threads.push_back(std::thread(&SocketHandler::send_task, this, sock));
    } 
}

void SocketHandler::timer_callback() 
{
    auto current_time = this->get_clock()->now();
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_state.position.theta);

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = current_time;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    
    transformStamped.transform.translation.x = robot_state.position.x;
    transformStamped.transform.translation.y = robot_state.position.y;
    transformStamped.transform.translation.z = 0.0;
    
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transformStamped);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = robot_state.position.x;
    odom.pose.pose.position.y = robot_state.position.y;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom);
}

void SocketHandler::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
{
    generic_msg_t generic_msg;
    generic_msg.cmd = SET_ROBOT_VELOCITY;
    generic_msg.velocity_msg.velocity.v_x = msg->linear.x;
    generic_msg.velocity_msg.velocity.v_y = msg->linear.y;
    generic_msg.velocity_msg.velocity.v_theta = msg->angular.z;

    for(int sock : client_sockets)
    {
        send(sock, &generic_msg, sizeof(generic_msg_t), 0);
    }
}