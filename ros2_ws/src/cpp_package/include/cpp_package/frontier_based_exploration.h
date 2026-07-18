#ifndef FRONTIER_BASED_EXPLORATION_H_
#define FRONTIER_BASED_EXPLORATION_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
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
            // Coordinate with the orchestrator: state != DETECT -> pause WFD (yield base control)
            state_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/orchestrator/state", 10, std::bind(&FrontierBasedExploration::stateCallback, this, std::placeholders::_1));
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

            // The robot pose must be taken in the map frame (via tf map->base_link), NOT /odom
            // directly: SLAM continuously corrects map->odom, so odom coords drift from the map -> wrong cell index.
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            is_navigating_ = true;
            timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&FrontierBasedExploration::exploreLoop, this));

            spin_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FrontierBasedExploration::spinScan, this));
        }

    private:

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void stateCallback(const std_msgs::msg::String::SharedPtr msg);
        bool getRobotMapPose(double &x, double &y) const;   // tf map->base_link
        int findNearestOpen(int start_index, int max_radius) const;  // nearest open cell to the robot
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
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
        bool paused_ = false;                 // orchestrator has taken control (state != DETECT)
        GoalHandle::SharedPtr current_goal_handle_;   // the running WFD goal (to cancel exactly our own goal)
        int current_goal_index_ = -1;
        bool awaiting_nav_result_ = false;    // waiting for NAV2 to return a goal result (distinct from the spin-scan phase)
        rclcpp::Time nav_start_time_;         // goal send time -> compute a timeout when NAV2 hangs
        uint64_t goal_seq_ = 0;               // monotonic goal id -> invalidate late callbacks from an old goal
        double start_yaw_ = 0.0;
        double last_yaw_ = 0.0;
        double total_rotated_ = 0.0;

        static constexpr size_t MIN_FRONTIER_SIZE = 10;
        static constexpr double SPIN_SPEED = 0.3;
        static constexpr double GOAL_TIMEOUT_SEC = 60.0;   // backstop: NAV2 returns no result -> drop the goal
};

#endif