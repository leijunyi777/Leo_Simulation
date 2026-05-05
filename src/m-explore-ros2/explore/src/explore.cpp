/*
 * explore.cpp — m-explore (ROS2) 探索节点实现
 *
 * 【给同步代码的同事：本文件相对上游 explore_lite 的主要工作逻辑】
 *
 * 1) 总体流程
 *    - 节点启动后连接 Nav2 的 navigate_to_pose Action，按 planner_frequency 定时调用 makePlan()。
 *    - 在代价地图上搜索 frontier，选代价最优且不在黑名单的目标，发给 Nav2；到达或失败后通过
 *      reachedGoal() 再触发下一轮规划（与原版一致）。
 *    - 发布 explore/status（ExploreStatus，transient_local），供上层监控：启动即 STARTED，
 *      正常扫完或熔断结束为 COMPLETE，手动 stop 为 PAUSED，resume 为 IN_PROGRESS 等。
 *    - 订阅 explore/resume（std_msgs/Bool）：true 调用 resume()，false 调用 stop()。
 *
 * 2) 探索“进度”指标（用于结束条件，非 SLAM 官方覆盖率）
 *    - 在全局代价地图栅格上统计：ratio = free_cells / (free_cells + unknown_cells)。
 *    - 启动后约 30s 内视为预热：若分母为 0 只打日志，避免早期空图误触发结束逻辑。
 *
 * 3) 面积/停滞熔断（makePlan 前半段）
 *    - 预热后若 ratio 几乎不变（|Δratio| ≤ 1e-4）持续过久：ratio < 85% 时限 40s，≥85% 时限 15s，
 *      超时认为扩图停滞 → 发 COMPLETE 并 stop(true)。
 *    - ratio > 95% → 认为探索率达标 → 发 COMPLETE 并 stop(true)。
 *
 * 4) 物理防卡死看门狗（makePlan 中取到机器人位姿后）
 *    - 若 5s 内位移 < 0.15m 且此前确实发过目标（prev_goal_ 非全零）：判定顶墙/卡住 →
 *      当前目标入黑名单、async_cancel_all_goals()、清空 prev_goal_ 记忆，本周期 return；
 *      下周期会选其它 frontier。注释里写的“20 秒”与代码 5.0 不一致时以代码为准。
 *
 * 5) 黑名单与“扫盲”重试
 *    - frontier_blacklist_：abort 的目标、progress_timeout_ 无进展的目标、看门狗拉黑等会加入。
 *    - goalOnBlacklist()：在代价地图分辨率下用格子容差（tolerace 格）判近邻。
 *    - 若当前无 frontier：黑名单非空时最多清空黑名单重试 3 次；若所有 frontier 都在黑名单：
 *      最多再清空 1 次。成功找到可用 frontier 后 blacklist_retry_count 归零。
 *
 * 6) 决策粘性（减少路口反复换目标）
 *    - 若新最优 frontier 与上一目标不同，但代价与 prev_goal_cost_ 相差不到约 25%，则本周期不
 *      发新 goal，让 Nav2 继续执行原目标。
 *
 * 7) 与原版一致的进展超时
 *    - 若离当前目标更近会刷新 last_progress_；若超过 progress_timeout_ 仍无进展且非 resuming_，
 *      将当前目标拉黑并递归 makePlan()。
 *    - 若本周期判定与上一目标 same_point（约 1cm 内），直接 return，不打断 Nav2。
 *
 * 8) ABORT 同点朝向重试（到达姿态）
 *    - Nav2 返回 ABORTED 时，不立即拉黑；在同一目标点上顺时针每次旋转 45° 变更 goal 朝向，
 *      每次间隔 1.5 秒，最多额外尝试 7 个朝向；若仍全部 ABORT 才拉黑并换下一个 frontier。
 *    - 重试进行期间会跳过 5s 看门狗，避免两个机制互相抢占导致误拉黑。
 *
 * 9) 结束与回原点
 *    - stop(finished_exploring)：finished_exploring==true 时不发 PAUSED（仅取消定时器与 goal）；
 *      若 return_to_init_ 且探索正常结束，会再发 RETURNING_TO_ORIGIN 并导航回启动时记录的位姿。
 *
 * 修改本文件时请同步更新本节，避免同事只看 diff 难以理解策略参数含义。
 */
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

// ABORT 后同一点朝向重试步长：每次顺时针 45°（负号在计算 yaw 时体现）。
inline static constexpr double kAbortYawStepRad = 0.7853981633974483;  // pi / 4

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_goal_cost_(0.0)
  , prev_distance_(0)
  , start_time_(this->now())
  , last_progress_change_time_(this->now())
  , last_progress_(this->now())
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

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
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
  // Start exploration right away
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
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
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
    // scale frontier according to its cost (costier frontiers will be smaller)
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

  // delete previous markers, which are now unused
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
auto now = this->now();
double uptime = (now - start_time_).seconds();

// 1. 获取地图数据（用于计算百分比）
auto* costmap2d = costmap_client_.getCostmap();
unsigned char* char_map = costmap2d->getCharMap();
unsigned int size = costmap2d->getSizeInCellsX() * costmap2d->getSizeInCellsY();
size_t free_cells = 0, unknown_cells = 0;

for (unsigned int i = 0; i < size; ++i) {
  if (char_map[i] == nav2_costmap_2d::FREE_SPACE) free_cells++;
  else if (char_map[i] == nav2_costmap_2d::NO_INFORMATION) unknown_cells++;
}

// 2. 核心熔断与停滞逻辑
if (free_cells + unknown_cells > 0) {
  double current_ratio = (double)free_cells / (free_cells + unknown_cells);
  
  // --- 打印进度 (每5秒打一次) ---
  RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 5000, "当前探索进度: %.2f%%", current_ratio * 100.0);

 
  // --- A. 30秒预热后的停滞检测 ---
  if (uptime > 30.0) {
    // 如果进度变化极其微小（设定万分之一阈值），说明没在扩图
    if (std::abs(current_ratio - last_progress_ratio_) > 0.0001) {
      last_progress_ratio_ = current_ratio;
      last_progress_change_time_ = now; // 重置“停滞闹钟”
    } else {
      double stalled_time = (now - last_progress_change_time_).seconds();

      // 🌟 动态计算下班时间阈值：85% 之前等 40 秒，85% 之后只等 15 秒
      double timeout_threshold = (current_ratio < 0.85) ? 40.0 : 15.0;

      if (stalled_time > timeout_threshold) {
        RCLCPP_WARN(logger_, "进度卡在 %.2f%% 超过 %.0f 秒，强行下班！", current_ratio * 100.0, timeout_threshold);
        auto status_msg = explore_lite_msgs::msg::ExploreStatus();
        status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
        status_pub_->publish(status_msg);
        stop(true);
        return;
      }
    }

    // --- B. 95% 面积熔断 ---
    if (current_ratio > 0.95) {
      RCLCPP_WARN(logger_, "探索率达标 (%.2f%%)，探索结束！", current_ratio * 100.0);
      auto status_msg = explore_lite_msgs::msg::ExploreStatus();
      status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
      status_pub_->publish(status_msg); 
      stop(true);
      return;
    }
  }
}else {
  // 保护期内，用来保护机器人，只打印一下预热提示
  RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 1000, "探索预热中... 剩余保护时间: %.1fs", 30.0 - uptime);
}
// --- 熔断器结束 ---  

  // 1. 获取机器人位姿并搜索边界
  auto pose = costmap_client_.getRobotPose();

  // =================================================================
  // 🌟 🐶 物理防卡死“看门狗”逻辑 (插入部分开始)
  // =================================================================
  if (!watchdog_initialized_) {
    watchdog_last_pose_ = pose;
    watchdog_last_time_ = now;
    watchdog_initialized_ = true;
  }

  // 计算移动距离和流逝时间
  double dx = pose.position.x - watchdog_last_pose_.position.x;
  double dy = pose.position.y - watchdog_last_pose_.position.y;
  double distance_moved = std::hypot(dx, dy);
  double time_elapsed = (now - watchdog_last_time_).seconds();

  if (distance_moved > 0.15) {
    // 移动超过 15cm，说明车还在正常跑，重置看门狗
    watchdog_last_pose_ = pose;
    watchdog_last_time_ = now;
  } else if (time_elapsed > 5.0 && !abort_yaw_retry_active_) {
    // 仅在“非 ABORT 朝向重试阶段”启用 5s 看门狗，避免两个机制互相打断。
    // 超过 5 秒钟，移动距离不到 15cm，判定为物理撞墙或卡在代价层！
    
    // 确保之前确实发过目标（坐标不全为0）
    if (prev_goal_.x != 0.0 || prev_goal_.y != 0.0) {
      RCLCPP_WARN(logger_, "⚠️ 检测到车辆物理卡死 (5秒未实质移动)！强行放弃并拉黑当前目标！");
      
      // 拉黑当前一直去不了的目标
      frontier_blacklist_.push_back(prev_goal_);
      
      // 打断 Nav2，让车马上停下不要再顶墙了。
      // 先清理 ABORT 重试状态，避免旧定时器后续误发同点重试目标。
      resetAbortYawRetry();
      move_base_client_->async_cancel_all_goals();

      // ==========================================
      // 🌟 修复 Bug 的关键补丁：清空上一次的目标记忆！
      // ==========================================
      prev_goal_.x = 0.0;
      prev_goal_.y = 0.0;
      prev_goal_cost_ = 0.0;
    }
    
    // 重置看门狗，避免死循环触发
    watchdog_last_pose_ = pose;
    watchdog_last_time_ = now;
    
    // 退出当前规划。下一秒 timer 会重新触发，并由于上一个点被拉黑，会去找新点！
    return; 
  }
  // =================================================================
  // 🌟 🐶 物理防卡死“看门狗”逻辑 (插入部分结束)
  // =================================================================



  if (abort_yaw_retry_active_) {
    // 同点朝向重试进行中：暂停 frontier 重选，等当前重试链给出结果。
    return;
  }

  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  // --- 🌟 防漏扫补丁：定义局部静态变量记录重试次数，防止无限死循环 ---
  static int blacklist_retry_count = 0;

  // 2. 如果没找到任何边界
  if (frontiers.empty()) {
    // 尝试抢救：黑名单里还有东西，且重试次数小于3次
    if (!frontier_blacklist_.empty() && blacklist_retry_count < 3) {
      RCLCPP_INFO(logger_, "没找到新边界，但黑名单有存货。清空黑名单，进行第 %d 次最后扫盲！", blacklist_retry_count + 1);
      frontier_blacklist_.clear();
      blacklist_retry_count++;
      return; // 直接返回，等下一个定时器周期重新规划
    }

    // 抢救无效或已达上限，正常结束
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // 3. 在 RViz 中可视化边界
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // 4. 寻找非黑名单的最优边界
  auto frontier_it =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });

  // 如果剩下的边界全在黑名单里
  if (frontier_it == frontiers.end()) {
    // 尝试抢救：清空黑名单再给一次机会
    if (blacklist_retry_count < 1) {
      RCLCPP_INFO(logger_, "剩下的边界全在黑名单中。清空黑名单，进行第 %d 次最后扫盲！", blacklist_retry_count + 1);
      frontier_blacklist_.clear();
      blacklist_retry_count++;
      return; // 直接返回，等下一个周期重新分配
    }

    // 抢救无效，正常结束
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // 🌟 只要成功找到一个不在黑名单里的有效边界，就把重试次数清零
  blacklist_retry_count = 0;

  // 5. 🌟 决策粘性逻辑 (防止路口抽搐横跳)
  geometry_msgs::msg::Point target_position = frontier_it->centroid;
  double current_frontier_cost = frontier_it->cost;
  bool same_goal = same_point(prev_goal_, target_position);

  // 如果新旧目标点不同，检查代价差距是否足够大 (15% 阈值)
  if (!same_goal && prev_goal_.x != 0.0) { 
    if (std::abs(current_frontier_cost - prev_goal_cost_) < std::abs(prev_goal_cost_ * 0.25)) {
      RCLCPP_DEBUG(logger_, "New goal not better enough, sticking to old goal");
      return; // 差距不大，继续坚持去原来的目标，不打断 Nav2
    }
  }

  // 6. 进度检查与超时处理
  if (!same_goal || prev_distance_ > frontier_it->min_distance) {
    // 我们有了新目标，或者我们离原目标更近了（取得了进展）
    last_progress_ = this->now();
    prev_distance_ = frontier_it->min_distance;
  }

  // 如果长时间没有进展（卡住了），把当前目标拉黑
  if ((this->now() - last_progress_ > tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "Adding current goal to black list due to timeout");
    makePlan();
    return;
  }

  if (resuming_) {
    resuming_ = false;
  }

  // 如果目标还是同一个，什么都不用做，让 Nav2 继续跑
  if (same_goal) {
    return;
  }

  // 🌟 更新记录，用于下一次的决策粘性比对
  prev_goal_ = target_position;
  prev_goal_cost_ = current_frontier_cost;

  // 7. 发送全新目标给 Nav2（默认 yaw = 0）
  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");
  sendExploreNavigateGoal(target_position, 0.0);
}

void Explore::sendExploreNavigateGoal(const geometry_msgs::msg::Point& position,
                                      double yaw_rad)
{
  // 统一在这里把 yaw 转为四元数，保证普通发点与 ABORT 重试发点走同一逻辑。
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = position;
  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = std::sin(yaw_rad * 0.5);
  goal.pose.pose.orientation.w = std::cos(yaw_rad * 0.5);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this, position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, position); 
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::resetAbortYawRetry()
{
  // 彻底清空重试链：停掉定时器并恢复初始索引。
  if (abort_yaw_retry_timer_) {
    abort_yaw_retry_timer_->cancel();
    abort_yaw_retry_timer_.reset();
  }
  abort_yaw_retry_active_ = false;
  abort_yaw_next_index_ = 1;
}

void Explore::scheduleAbortYawRetryTimer()
{
  if (abort_yaw_retry_timer_) {
    abort_yaw_retry_timer_->cancel();
    abort_yaw_retry_timer_.reset();
  }

  abort_yaw_retry_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1500), [this]() {
        // 若外部（stop/cancel/success）已经终止重试链，直接清理退出。
        if (!abort_yaw_retry_active_) {
          resetAbortYawRetry();
          return;
        }

        int retry_index = abort_yaw_next_index_;
        if (retry_index < 1 || retry_index > 7) {
          abort_yaw_retry_timer_->cancel();
          abort_yaw_retry_timer_.reset();
          return;
        }

        const double retry_yaw = -static_cast<double>(retry_index) * kAbortYawStepRad;
        RCLCPP_INFO(logger_,
                    "ABORT recovery: retry same goal yaw step %d/7 (%.2f deg)",
                    retry_index, retry_yaw * 180.0 / 3.14159265358979323846);

        // 一次性定时器：本次只发一个朝向，下一次由新的 ABORT 回调再安排。
        abort_yaw_retry_timer_->cancel();
        abort_yaw_retry_timer_.reset();
        sendExploreNavigateGoal(abort_yaw_retry_position_, retry_yaw);
      });
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
  // 🌟 将 5 改成 15，扩大黑名单的判定半径，避免在死胡同里一点一点地试探
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
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
      // 一旦成功，立即结束任何未完成的同点朝向重试链。
      resetAbortYawRetry();
      break;
    case rclcpp_action::ResultCode::ABORTED: {
      RCLCPP_WARN(logger_, "Goal was aborted");

      if (abort_yaw_retry_active_) {
        if (same_point(frontier_goal, abort_yaw_retry_position_)) {
          // 同一目标点再次 ABORT：推进到下一个朝向重试。
          abort_yaw_next_index_++;
          if (abort_yaw_next_index_ > 7) {
            RCLCPP_WARN(logger_,
                        "ABORT recovery exhausted 7 yaw retries, blacklisting goal");
            // 8 个朝向（初始 + 7 次重试）都失败：拉黑该点并转下一 frontier。
            frontier_blacklist_.push_back(abort_yaw_retry_position_);
            resetAbortYawRetry();
            makePlan();
            return;
          }
          scheduleAbortYawRetryTimer();
          return;
        }

        // 异常保护：若回调目标点不一致，重置旧重试链并以当前 ABORT 点重新开始。
        resetAbortYawRetry();
      }

      // 首次 ABORT：进入同点朝向重试链（第 1 步对应 -45°）。
      abort_yaw_retry_active_ = true;
      abort_yaw_retry_position_ = frontier_goal;
      abort_yaw_next_index_ = 1;
      scheduleAbortYawRetryTimer();
      return;
    }
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      // 被外部打断时务必清理重试链，避免 timer 残留。
      resetAbortYawRetry();
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
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
  
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }
  
  // stop() 覆盖手动暂停、FSM 打断和探索结束收尾，统一清理重试链。
  resetAbortYawRetry();
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
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
