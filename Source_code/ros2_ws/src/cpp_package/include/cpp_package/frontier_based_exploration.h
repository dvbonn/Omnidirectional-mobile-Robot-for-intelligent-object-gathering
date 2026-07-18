#ifndef FRONTIER_BASED_EXPLORATION_H_
#define FRONTIER_BASED_EXPLORATION_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <queue>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>

class FrontierBasedExploration : public rclcpp::Node
{
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        enum CellState
        {
            NONE,
            MAP_OPEN,
            MAP_CLOSE,
            FRONTIER_OPEN,
            FRONTIER_CLOSE
        };

        struct Frontier
        {
            std::vector<int> cells;
            double median_x = 0.0;
            double median_y = 0.0;
            double distance = 0.0;
        };

        FrontierBasedExploration() : Node("frontier_based_exploration")
        {
            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&FrontierBasedExploration::mapCallback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10, std::bind(&FrontierBasedExploration::odomCallback, this, std::placeholders::_1));
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
            
            is_navigating_ = true;
            timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&FrontierBasedExploration::exploreLoop, this));

            spin_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FrontierBasedExploration::spinScan, this));
        }

    private:

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        int worldToIndex(double x, double y) const;
        void indexToWorld(int index, double &x, double &y) const;
        std::vector<int> getNeighbors(int index) const;
        bool isOpenSpace(int index) const;
        bool isUnknown(int index) const;
        bool isFrontierPoint(int index) const;
        bool hasOpenSpaceNeighbor(int index) const;
        Frontier extractFrontier(int start_index);
        void calcMedian(Frontier &f);
        std::vector<Frontier> detectFrontiers();
        void exploreLoop();
        void sendGoal(double x, double y, int goal_index);
        void spinScan();

        void publishFrontierMarkers(const std::vector<Frontier>& frontiers);
        void publishGoalMarker(double x, double y);
        
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr spin_timer_;

        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double robot_yaw_ = 0.0;

        std::vector<CellState> cell_states_;
        std::set<int> blacklist_;

        bool is_navigating_ = false;
        bool is_spinning_ = false;
        int current_goal_index_ = -1;
        double start_yaw_ = 0.0;
        double last_yaw_ = 0.0;
        double total_rotated_ = 0.0;

        static constexpr size_t MIN_FRONTIER_SIZE = 10;
        static constexpr double SPIN_SPEED = 0.3;
};

#endif