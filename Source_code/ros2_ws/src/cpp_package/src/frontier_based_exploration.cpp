#include "cpp_package/frontier_based_exploration.h"

void FrontierBasedExploration::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = msg;
}

void FrontierBasedExploration::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
}

int FrontierBasedExploration::worldToIndex(double x, double y) const
{
    auto& info = map_->info;
    int col = static_cast<int>((x - info.origin.position.x) / info.resolution);
    int row = static_cast<int>((y - info.origin.position.y) / info.resolution);
    if(col < 0 || col >= static_cast<int>(info.width) || 
       row < 0 || row >= static_cast<int>(info.height))
    {
        return -1;
    }
    return row * info.width + col;
}

void FrontierBasedExploration::indexToWorld(int index, double &x, double &y) const
{
    auto& info = map_->info;
    int col = index % info.width;
    int row = index / info.width;
    x = info.origin.position.x + (col + 0.5) * info.resolution;
    y = info.origin.position.y + (row + 0.5) * info.resolution;
}

std::vector<int> FrontierBasedExploration::getNeighbors(int index) const
{
    std::vector<int> neighbors;
    auto& info = map_->info;
    int col = index % info.width;
    int row = index / info.width;

    for(int dr = -1; dr <= 1; ++dr)
    {
        for(int dc = -1; dc <= 1; ++dc)
        {
            if(dr == 0 && dc == 0) continue;
            int new_col = col + dc;
            int new_row = row + dr;
            if(new_col < 0 || new_col >= (int)info.width) continue;
            if(new_row < 0 || new_row >= (int)info.height) continue;
            neighbors.push_back(new_row * info.width + new_col);
        }
    }
    return neighbors;
}

bool FrontierBasedExploration::isOpenSpace(int index) const
{
    return map_->data[index] == 0;
}

bool FrontierBasedExploration::isUnknown(int index) const
{
    return map_->data[index] == -1;
}

bool FrontierBasedExploration::isFrontierPoint(int index) const
{
    if(!isOpenSpace(index)) return false;
    for(int neighbor : getNeighbors(index))
    {
        if(isUnknown(neighbor))
        {
            return true;
        }
    }
    return false;
}

bool FrontierBasedExploration::hasOpenSpaceNeighbor(int index) const
{
    for(int neighbor : getNeighbors(index))
    {
        if(isOpenSpace(neighbor))
        {
            return true;
        }
    }
    return false;
}

FrontierBasedExploration::Frontier FrontierBasedExploration::extractFrontier(int start_index)
{
    FrontierBasedExploration::Frontier f;
    std::queue<int> queue_f;

    queue_f.push(start_index);
    cell_states_[start_index] = FRONTIER_OPEN;

    while(!queue_f.empty())
    {
        int q = queue_f.front();
        queue_f.pop();

        if(cell_states_[q] == MAP_CLOSE ||
           cell_states_[q] == FRONTIER_CLOSE)
        {
            continue;
        }

        if(isFrontierPoint(q))
        {
            f.cells.push_back(q);

            for(int neighbor : getNeighbors(q))
            {
                if(cell_states_[neighbor] != FRONTIER_OPEN &&
                   cell_states_[neighbor] != FRONTIER_CLOSE &&
                   cell_states_[neighbor] != MAP_CLOSE)
                {
                    queue_f.push(neighbor);
                    cell_states_[neighbor] = FRONTIER_OPEN;
                }
            }
        }
        cell_states_[q] = FRONTIER_CLOSE;
    }

    for(int index : f.cells)
    {
        cell_states_[index] = MAP_CLOSE;
    }

    calcMedian(f);

    return f;
}

std::vector<FrontierBasedExploration::Frontier> FrontierBasedExploration::detectFrontiers()
{
    std::vector<FrontierBasedExploration::Frontier> frontiers;
    if(!map_) return frontiers;

    int n = map_->info.width * map_->info.height;

    cell_states_.assign(n, NONE);

    int start = worldToIndex(robot_x_, robot_y_);
    if(start < 0 || !isOpenSpace(start))
    {
        RCLCPP_WARN(this->get_logger(), "Robot is not in open space. Cannot detect frontiers.");
        return frontiers;
    }

    std::queue<int> queue_m;
    queue_m.push(start);
    cell_states_[start] = MAP_OPEN;
    
    while(!queue_m.empty())
    {
        int p = queue_m.front();
        queue_m.pop();

        if(cell_states_[p] == MAP_CLOSE)
        {
            continue;
        }

        if(isFrontierPoint(p))
        {
            FrontierBasedExploration::Frontier f = extractFrontier(p);
            if(f.cells.size() >= MIN_FRONTIER_SIZE)
            {
                f.distance = std::hypot(f.median_x - robot_x_, f.median_y - robot_y_);
                frontiers.push_back(f);
            }
        }

        for(int neighbor : getNeighbors(p))
        {
            if(cell_states_[neighbor] != MAP_OPEN &&
               cell_states_[neighbor] != MAP_CLOSE &&
               hasOpenSpaceNeighbor(neighbor))
            {
                queue_m.push(neighbor);
                cell_states_[neighbor] = MAP_OPEN;
            }
        }
        cell_states_[p] = MAP_CLOSE;
    }
    return frontiers;
}

void FrontierBasedExploration::calcMedian(Frontier &f)
{
    double x_c = 0.0, y_c = 0.0;
    std::vector<double> xs, ys;
    for(int index : f.cells)
    {
        double x, y;
        indexToWorld(index, x, y);
        xs.push_back(x);
        ys.push_back(y);
        x_c += x;
        y_c += y;
    }

    x_c /= f.cells.size();
    y_c /= f.cells.size();

    double min_dist_sq = std::numeric_limits<double>::max();
    double nearest_x = 0.0, nearest_y = 0.0;
    
    for(size_t i = 0; i < xs.size(); i++)
    {
        double dx = xs[i] - x_c;
        double dy = ys[i] - y_c;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            nearest_x = xs[i];
            nearest_y = ys[i];
        }
    }

    f.median_x = nearest_x;
    f.median_y = nearest_y;
}

void FrontierBasedExploration::exploreLoop()
{
    if(is_navigating_ || !map_) return;
    
    auto frontiers = detectFrontiers();

    if(frontiers.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No frontiers detected. Exploration complete.");
        timer_->cancel();
        return;
    }

    std::sort(frontiers.begin(), frontiers.end(), [](const Frontier &a, const Frontier &b) {
        return a.distance < b.distance;
    });

    for(auto& f : frontiers)
    {
        int goal_index = worldToIndex(f.median_x, f.median_y);
        if(blacklist_.count(goal_index)) continue;

        RCLCPP_INFO(this->get_logger(), "Frontier: (%.2f, %.2f),Size: %lu, Distance: %.2f", f.median_x, f.median_y, f.cells.size(), f.distance);
        
        sendGoal(f.median_x, f.median_y, goal_index);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "All frontiers are blacklisted");
}

void FrontierBasedExploration::sendGoal(double x, double y, int goal_index)
{
    if(!nav_client_->wait_for_action_server(std::chrono::seconds(3)))
    {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available");
        return;
    }

    current_goal_index_ = goal_index;

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback = 
        [this](std::shared_future<GoalHandle::SharedPtr> future) 
        {
            auto gh = future.get();
            if(!gh)
            {
                RCLCPP_WARN(this->get_logger(), "Goal was rejected by the action server");
                is_navigating_ = false;
                blacklist_.insert(current_goal_index_);
                exploreLoop();
            }
        };
    
    opts.result_callback = 
        [this](const GoalHandle::WrappedResult &result)
        {
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
                spin_timer_->reset();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
                is_navigating_ = false;
                blacklist_.insert(current_goal_index_);
                exploreLoop();
            }
        };

    is_navigating_ = true;
    nav_client_->async_send_goal(goal, opts);
}

void FrontierBasedExploration::spinScan()
{
    geometry_msgs::msg::Twist cmd;
    if(!is_spinning_)
    {
        is_spinning_ = true;
        start_yaw_ = robot_yaw_;
        total_rotated_ = 0.0;
        last_yaw_ = robot_yaw_;
    }

    double delta = robot_yaw_ - last_yaw_;

    if(delta > M_PI)
    {
        delta -= 2 * M_PI;
    }
    else if(delta < -M_PI)
    {
        delta += 2 * M_PI;
    }

    total_rotated_ += std::abs(delta);
    last_yaw_ = robot_yaw_;

    RCLCPP_INFO(this->get_logger(), "Spinning... rotated: %.2f / %.2f rad",
        total_rotated_, 2 * M_PI);

    if(total_rotated_ >= M_PI * 2)
    {
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
        is_spinning_ = false;
        RCLCPP_INFO(this->get_logger(), "Spin scan complete");
        spin_timer_->cancel();
        is_navigating_ = false;       
        return;
    }

    cmd.angular.z = SPIN_SPEED;
    cmd_vel_pub_->publish(cmd);
}