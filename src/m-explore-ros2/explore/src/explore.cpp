


 #include <explore/explore.h>

 #include <thread>
 
 inline static bool same_point(const geometry_msgs::msg::Point& one,
                               const geometry_msgs::msg::Point& two)
 {
   double dx = one.x - two.x;
   double dy = one.y - two.y;
   double dist = sqrt(dx * dx + dy * dy);
   return dist < 0.01;
 }
 
 namespace explore
 {
 Explore::Explore()
   : Node("explore_node")
   , logger_(this->get_logger())
   , tf_buffer_(this->get_clock())
   , tf_listener_(tf_buffer_)
   , costmap_client_(*this, &tf_buffer_)
   , prev_distance_(0)
   , last_markers_count_(0)
 {
   double timeout;
   double min_frontier_size;
   this->declare_parameter<float>("planner_frequency", 1.0);
   this->declare_parameter<float>("progress_timeout", 30.0);
   this->declare_parameter<bool>("visualize", false);
   this->declare_parameter<float>("potential_scale", 1e-3);
   this->declare_parameter<float>("orientation_scale", 0.0);
   this->declare_parameter<float>("gain_scale", 1.0);
   this->declare_parameter<float>("min_frontier_size", 0.5);
   this->declare_parameter<bool>("return_to_init", false);
 
   this->get_parameter("planner_frequency", planner_frequency_);
   this->get_parameter("progress_timeout", timeout);
   this->get_parameter("visualize", visualize_);
   this->get_parameter("potential_scale", potential_scale_);
   this->get_parameter("orientation_scale", orientation_scale_);
   this->get_parameter("gain_scale", gain_scale_);
   this->get_parameter("min_frontier_size", min_frontier_size);
   this->get_parameter("return_to_init", return_to_init_);
   this->get_parameter("robot_base_frame", robot_base_frame_);
 
   progress_timeout_ = timeout;
   move_base_client_ =
       rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
           this, ACTION_NAME);
 
   search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                  potential_scale_, gain_scale_,
                                                  min_frontier_size, logger_);
 
   if (visualize_) {
     marker_array_publisher_ =
         this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                      "frontier"
                                                                      "s",
                                                                      10);
   }
 
  // 探索状态发布器
   rclcpp::QoS status_qos(10);
   status_qos.transient_local();
   status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);
 
  // 恢复或停止探索订阅
   resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
       "explore/resume", 10,
       std::bind(&Explore::resumeCallback, this, std::placeholders::_1));
 
   RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
   move_base_client_->wait_for_action_server();
   RCLCPP_INFO(logger_, "Connected to move_base nav2 server");
 
   if (return_to_init_) {
     RCLCPP_INFO(logger_, "Getting initial pose of the robot");
     geometry_msgs::msg::TransformStamped transformStamped;
     std::string map_frame = costmap_client_.getGlobalFrameID();
     try {
       transformStamped = tf_buffer_.lookupTransform(
           map_frame, robot_base_frame_, tf2::TimePointZero);
       initial_pose_.position.x = transformStamped.transform.translation.x;
       initial_pose_.position.y = transformStamped.transform.translation.y;
       initial_pose_.orientation = transformStamped.transform.rotation;
     } catch (tf2::TransformException& ex) {
       RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                    map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
       return_to_init_ = false;
     }
   }
 
   exploring_timer_ = this->create_wall_timer(
       std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
       [this]() { makePlan(); });
  // 立即开始探索
   auto status_msg = explore_lite_msgs::msg::ExploreStatus(); 
   status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
   status_pub_->publish(status_msg);   
   makePlan();
 }
 
 Explore::~Explore()
 {
   stop();
 }
 
 void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
 {
   if (msg->data) {
     resume();
   } else {
     stop();
   }
 }
 
 void Explore::visualizeFrontiers(
     const std::vector<frontier_exploration::Frontier>& frontiers)
 {
   std_msgs::msg::ColorRGBA blue;
   blue.r = 0;
   blue.g = 0;
   blue.b = 1.0;
   blue.a = 1.0;
   std_msgs::msg::ColorRGBA red;
   red.r = 1.0;
   red.g = 0;
   red.b = 0;
   red.a = 1.0;
   std_msgs::msg::ColorRGBA green;
   green.r = 0;
   green.g = 1.0;
   green.b = 0;
   green.a = 1.0;
 
   RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
   visualization_msgs::msg::MarkerArray markers_msg;
   std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
   visualization_msgs::msg::Marker m;
 
   m.header.frame_id = costmap_client_.getGlobalFrameID();
   m.header.stamp = this->now();
   m.ns = "frontiers";
   m.scale.x = 1.0;
   m.scale.y = 1.0;
   m.scale.z = 1.0;
   m.color.r = 0;
   m.color.g = 0;
   m.color.b = 255;
   m.color.a = 255;
  // 永久存在
 #ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // 在 galactic 中已弃用（警告）
 #elif DASHING
  m.lifetime = rclcpp::Duration(0);  // 在 galactic 中已弃用（警告）
 #else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy 之后
 #endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // 在 galactic 中有建议
   m.frame_locked = true;
 
  // 加权后的前沿点总是已排序
   double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;
 
   m.action = visualization_msgs::msg::Marker::ADD;
   size_t id = 0;
   for (auto& frontier : frontiers) {
     m.type = visualization_msgs::msg::Marker::POINTS;
     m.id = int(id);
    // m.pose.position = {}; // 编译警告
     m.scale.x = 0.1;
     m.scale.y = 0.1;
     m.scale.z = 0.1;
     m.points = frontier.points;
     if (goalOnBlacklist(frontier.centroid)) {
       m.color = red;
     } else {
       m.color = blue;
     }
     markers.push_back(m);
     ++id;
     m.type = visualization_msgs::msg::Marker::SPHERE;
     m.id = int(id);
     m.pose.position = frontier.initial;
    // 按代价缩放前沿点（代价越高，尺度越小）
     double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
     m.scale.x = scale;
     m.scale.y = scale;
     m.scale.z = scale;
     m.points = {};
     m.color = green;
     markers.push_back(m);
     ++id;
   }
   size_t current_markers_count = markers.size();
 
  // 删除先前的标记（当前已不再使用）
   m.action = visualization_msgs::msg::Marker::DELETE;
   for (; id < last_markers_count_; ++id) {
     m.id = int(id);
     markers.push_back(m);
   }
 
   last_markers_count_ = current_markers_count;
   marker_array_publisher_->publish(markers_msg);
 }
 
 void Explore::makePlan()
 {
  // 查找前沿点
   auto pose = costmap_client_.getRobotPose();
  // 获取按代价排序后的前沿点
   auto frontiers = search_.searchFrom(pose.position);
   RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
   for (size_t i = 0; i < frontiers.size(); ++i) {
     RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
   }
 
   if (frontiers.empty()) {
     RCLCPP_WARN(logger_, "No frontiers found, stopping.");
     auto status_msg = explore_lite_msgs::msg::ExploreStatus();
     status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
     status_pub_->publish(status_msg);
     stop(true);
     return;
   }
 
  // 以可视化标记的形式发布前沿点
   if (visualize_) {
     visualizeFrontiers(frontiers);
   }
 
  // 查找不在黑名单中的前沿点
   auto frontier =
       std::find_if_not(frontiers.begin(), frontiers.end(),
                        [this](const frontier_exploration::Frontier& f) {
                          return goalOnBlacklist(f.centroid);
                        });
   if (frontier == frontiers.end()) {
     RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
     auto status_msg = explore_lite_msgs::msg::ExploreStatus();
     status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
     status_pub_->publish(status_msg);
     stop(true);
     return;
   }
   // 恢复使用 centroid 作为目标点，保证导航有足够的距离
   geometry_msgs::msg::Point target_position = frontier->centroid;
 
  // 若长时间没有进展则超时
   bool same_goal = same_point(prev_goal_, target_position);
 
   prev_goal_ = target_position;
   if (!same_goal || prev_distance_ > frontier->min_distance) {
    // 目标发生变化或我们取得了进展
     last_progress_ = this->now();
     prev_distance_ = frontier->min_distance;
   }
  // 若长时间没有进展则加入黑名单
   if ((this->now() - last_progress_ >
       tf2::durationFromSec(progress_timeout_)) && !resuming_) {
     frontier_blacklist_.push_back(target_position);
     RCLCPP_DEBUG(logger_, "Adding current goal to black list");
     makePlan();
     return;
   }
 
  // 确保只有第一次调用 makePlan 时将 resuming 设为 true
   if (resuming_) {
     resuming_ = false;
   }
 
  // 如果仍在追踪同一目标则无需处理
   if (same_goal) {
     return;
   }
 
   RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");
 
  // 如果有新的目标要追踪，则将目标发送给 move_base
   auto goal = nav2_msgs::action::NavigateToPose::Goal();
   goal.pose.pose.position = target_position;
   goal.pose.pose.orientation.w = 1.;
   goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
   goal.pose.header.stamp = this->now();
 
   auto send_goal_options =
       rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
   // send_goal_options.goal_response_callback =
  // std::bind(&Explore::goal_response_callback, this, _1);
   // send_goal_options.feedback_callback =
   //   std::bind(&Explore::feedback_callback, this, _1, _2);
   send_goal_options.result_callback =
       [this,
        target_position](const NavigationGoalHandle::WrappedResult& result) {
         reachedGoal(result, target_position);
       };
   move_base_client_->async_send_goal(goal, send_goal_options);
 }
 
 void Explore::returnToInitialPose()
 {
   RCLCPP_INFO(logger_, "Returning to initial pose.");
   auto status_msg = explore_lite_msgs::msg::ExploreStatus();
   status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
   status_pub_->publish(status_msg);
   
   auto goal = nav2_msgs::action::NavigateToPose::Goal();
   goal.pose.pose.position = initial_pose_.position;
   goal.pose.pose.orientation = initial_pose_.orientation;
   goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
   goal.pose.header.stamp = this->now();
 
   auto send_goal_options =
       rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
   send_goal_options.result_callback = 
       [this](const NavigationGoalHandle::WrappedResult& result) {
         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
           auto status_msg = explore_lite_msgs::msg::ExploreStatus();
           status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
           status_pub_->publish(status_msg);
           RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
         }
       };
   move_base_client_->async_send_goal(goal, send_goal_options);
 }
 bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
 {
   constexpr static size_t tolerace = 5;
   nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();
 
  // 检查该目标是否在我们追踪的黑名单中
   for (auto& frontier_goal : frontier_blacklist_) {
     double x_diff = fabs(goal.x - frontier_goal.x);
     double y_diff = fabs(goal.y - frontier_goal.y);
 
     if (x_diff < tolerace * costmap2d->getResolution() &&
         y_diff < tolerace * costmap2d->getResolution())
       return true;
   }
   return false;
 }
 
 void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                           const geometry_msgs::msg::Point& frontier_goal)
 {
   switch (result.code) {
     case rclcpp_action::ResultCode::SUCCEEDED:
       RCLCPP_DEBUG(logger_, "Goal was successful");
       break;
     case rclcpp_action::ResultCode::ABORTED:
       RCLCPP_DEBUG(logger_, "Goal was aborted");
       frontier_blacklist_.push_back(frontier_goal);
       RCLCPP_DEBUG(logger_, "Adding current goal to black list");
      // 如果被中止，可能是因为找到了另一个前沿目标；
      // 因此直接返回，不再重新规划
       return;
     case rclcpp_action::ResultCode::CANCELED:
       RCLCPP_DEBUG(logger_, "Goal was canceled");
      // 如果目标被取消，可能是因为通过话题停止了探索；
      // 此时不要再生成新的规划
       return;
     default:
       RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
       break;
   }
  // 不管规划频率如何，立即寻找新的目标。
  // 通过定时器执行可避免 move_base_client 死锁：
  // 这里的回调是 sendGoal 的回调，而 sendGoal 是在 makePlan 中触发的；
  // 因此定时器需要一直存活到回调执行完成为止。
   // oneshot_ = relative_nh_.createTimer(
   //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
   //     true);
 
  // 鉴于 ros2 的单线程执行器特性，我认为定时器其实不需要。
   makePlan();
 }
 
 void Explore::start()
 {
   RCLCPP_INFO(logger_, "Exploration started.");
   auto status_msg = explore_lite_msgs::msg::ExploreStatus();
   status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
   status_pub_->publish(status_msg);
 }
 
 void Explore::stop(bool finished_exploring)
 {
   RCLCPP_INFO(logger_, "Exploration stopped.");
   
  // 只有在手动停止（未完成探索）时才发布暂停状态
   if (!finished_exploring) {
     auto status_msg = explore_lite_msgs::msg::ExploreStatus();
     status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
     status_pub_->publish(status_msg);
   }
   
   move_base_client_->async_cancel_all_goals();
   exploring_timer_->cancel();
 
   if (return_to_init_ && finished_exploring) {
     returnToInitialPose();
   }
 }
 
 void Explore::resume()
 {
   resuming_ = true;
   RCLCPP_INFO(logger_, "Exploration resuming.");
   auto status_msg = explore_lite_msgs::msg::ExploreStatus();
   status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
   status_pub_->publish(status_msg);
  // 重新启用定时器
   exploring_timer_->reset();
  // 立即恢复并开始规划
   makePlan();
 }
 
 }  // namespace explore
 
 int main(int argc, char** argv)
 {
   rclcpp::init(argc, argv);
  // ROS1 代码
   /*
   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                      ros::console::levels::Debug)) {
     ros::console::notifyLoggerLevelsChanged();
   } */
   rclcpp::spin(
      std::make_shared<explore::Explore>());  // 是否需要 std::move(std::make_unique)？
   rclcpp::shutdown();
   return 0;
 }
 