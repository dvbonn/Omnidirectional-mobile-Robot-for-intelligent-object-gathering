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

void FrontierBasedExploration::stateCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // Orchestrator in DETECT = scanning for objects -> WFD is allowed to run.
    // Any other state (LOCATE/NAVIGATE/APPROACH/ARRIVED) = orchestrator owns the base -> stop WFD.
    const bool should_pause = (msg->data != "DETECT");

    if(should_pause && !paused_)
    {
        paused_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Orchestrator active (state=%s) -> PAUSE WFD (cancel goal, stop spin scan)",
            msg->data.c_str());

        // Cancel only WFD's own goal, do not touch a goal the orchestrator sent to NAV2.
        if(current_goal_handle_)
        {
            nav_client_->async_cancel_goal(current_goal_handle_);
        }
        // Invalidate the pending goal: ++goal_seq_ so callbacks (even a not-yet-accepted goal) are
        // ignored, and clear the waiting flag so exploreLoop does not time out the yielded goal.
        ++goal_seq_;
        current_goal_handle_ = nullptr;
        awaiting_nav_result_ = false;

        // Stop the in-progress spin scan & block any leftover spin command on the base.
        is_spinning_ = false;
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
    else if(!should_pause && paused_)
    {
        paused_ = false;
        // Unlock is_navigating_: the WFD goal (if any) was cancelled at pause; if it happened to
        // return SUCCEEDED during the pause, the success branch returned early and left the flag stuck true.
        // Reset it so the 3s exploreLoop timer definitely resumes exploration.
        is_navigating_ = false;
        RCLCPP_INFO(this->get_logger(), "Orchestrator back to DETECT -> RESUME WFD");
    }
}

bool FrontierBasedExploration::getRobotMapPose(double &x, double &y) const
{
    // Robot pose in the map frame = tf map->base_link (SLAM: map->odom, base_bridge: odom->base_link).
    // Using /odom directly would be wrong because SLAM continuously corrects map->odom.
    try
    {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        x = tf.transform.translation.x;
        y = tf.transform.translation.y;
        return true;
    }
    catch(const tf2::TransformException &)
    {
        return false;
    }
}

int FrontierBasedExploration::findNearestOpen(int start_index, int max_radius) const
{
    // The robot may sit on an unknown cell (the Astra is blind <~0.6m, so the cell right under the
    // robot is often not yet raytraced to free), while open space is right next to it. Find the
    // nearest open cell by increasing radius to start the frontier BFS from. -1 if no open cell in range.
    if(start_index < 0) return -1;
    if(isOpenSpace(start_index)) return start_index;
    auto& info = map_->info;
    int scol = start_index % info.width;
    int srow = start_index / info.width;
    for(int r = 1; r <= max_radius; ++r)
    {
        for(int dr = -r; dr <= r; ++dr)
        {
            for(int dc = -r; dc <= r; ++dc)
            {
                if(std::abs(dr) != r && std::abs(dc) != r) continue;  // only the border of ring r
                int c = scol + dc, rr = srow + dr;
                if(c < 0 || c >= (int)info.width || rr < 0 || rr >= (int)info.height) continue;
                int idx = rr * info.width + c;
                if(isOpenSpace(idx)) return idx;
            }
        }
    }
    return -1;
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

    double rx, ry;
    if(!getRobotMapPose(rx, ry))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "No tf map->base_link yet (SLAM/tf not ready).");
        return frontiers;
    }

    int n = map_->info.width * map_->info.height;

    cell_states_.assign(n, NONE);

    int start = worldToIndex(rx, ry);
    if(start < 0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Robot outside map bounds @(%.2f,%.2f).", rx, ry);
        return frontiers;
    }
    // The robot cell may be unknown (Astra blind up close) -> start the BFS from the nearest open cell (~0.9m @res0.03).
    start = findNearestOpen(start, 30);
    if(start < 0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "No open space within ~0.9m around the robot @map (%.2f,%.2f) - waiting for SLAM to raytrace.",
            rx, ry);
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
                f.distance = std::hypot(f.median_x - rx, f.median_y - ry);
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
    if(paused_ || !map_) return;

    // NAV2 goal timeout: while WAITING FOR A NAV RESULT (distinct from the spin-scan phase), if it
    // exceeds GOAL_TIMEOUT_SEC -> treat NAV2 as hung (accepted the goal but never returns a result).
    // Cancel the goal, blacklist it, unlock to go to another frontier; ++goal_seq_ so a late callback
    // for this goal is ignored and does not corrupt the next goal's state.
    if(awaiting_nav_result_)
    {
        if((this->now() - nav_start_time_).seconds() > GOAL_TIMEOUT_SEC)
        {
            RCLCPP_WARN(this->get_logger(),
                "NAV2 goal hung > %.0fs -> cancel + blacklist, try another frontier", GOAL_TIMEOUT_SEC);
            ++goal_seq_;
            awaiting_nav_result_ = false;
            is_navigating_ = false;
            if(current_goal_handle_) nav_client_->async_cancel_goal(current_goal_handle_);
            current_goal_handle_ = nullptr;
            blacklist_.insert(current_goal_index_);
        }
        return;   // still waiting (or just timed out) -> let a later tick send a new goal
    }

    if(is_navigating_) return;   // spin scanning (spinScan) -> do not send a new goal yet

    auto frontiers = detectFrontiers();

    if(frontiers.empty())
    {
        // Do NOT cancel the timer permanently: frontiers can be empty TRANSIENTLY - the map is not
        // built yet (SLAM /scan ~2Hz in both mode), the robot just resumed after a pause, or the
        // robot cell is not yet open space. Cancelling here = killing exploration for the whole base
        // session. Keep the timer so it retries as the map grows.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No frontier yet - waiting for the map to grow / retrying (not stopping exploration).");
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
    const uint64_t my_seq = ++goal_seq_;   // this goal's id (guards against stale callbacks)

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
        [this, my_seq](std::shared_future<GoalHandle::SharedPtr> future)
        {
            auto gh = future.get();
            if(my_seq != goal_seq_)
            {
                // This goal was dropped (timeout/pause already moved on to another goal). If NAV2
                // still accepted it, cancel now so it does not run wild in parallel.
                if(gh) nav_client_->async_cancel_goal(gh);
                return;
            }
            if(!gh)
            {
                RCLCPP_WARN(this->get_logger(), "Goal was rejected by the action server");
                current_goal_handle_ = nullptr;
                awaiting_nav_result_ = false;
                is_navigating_ = false;
                if(paused_) return;
                blacklist_.insert(current_goal_index_);
                exploreLoop();
                return;
            }
            current_goal_handle_ = gh;   // store to cancel exactly WFD's goal when we need to stop
            if(paused_)
            {
                // Pause arrived BEFORE the goal was accepted -> cancel now,
                // otherwise the WFD goal runs in parallel and contends for NAV2/cmd_vel with the orchestrator.
                nav_client_->async_cancel_goal(current_goal_handle_);
            }
        };

    opts.result_callback =
        [this, my_seq](const GoalHandle::WrappedResult &result)
        {
            if(my_seq != goal_seq_) return;   // callback for a dropped goal (timeout/pause) -> ignore
            current_goal_handle_ = nullptr;
            awaiting_nav_result_ = false;
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
                if(paused_) return;      // control was yielded -> no spin scan
                spin_timer_->reset();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
                is_navigating_ = false;
                if(paused_) return;      // goal cancelled due to pause -> do not blacklist, do not keep exploring
                blacklist_.insert(current_goal_index_);
                exploreLoop();
            }
        };

    is_navigating_ = true;
    awaiting_nav_result_ = true;
    nav_start_time_ = this->now();
    nav_client_->async_send_goal(goal, opts);
}

void FrontierBasedExploration::spinScan()
{
    if(paused_) return;   // orchestrator is driving the base -> do not publish a spin cmd_vel

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
