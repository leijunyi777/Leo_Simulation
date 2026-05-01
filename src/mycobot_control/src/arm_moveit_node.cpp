#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "my_robot_interfaces/msg/cam_arm_pose.hpp"
#include "my_robot_interfaces/msg/gripper_state.hpp"

using std::placeholders::_1;

class ArmMoveItNode : public rclcpp::Node
{
public:
    ArmMoveItNode() : Node("arm_moveit_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动，等待初始化 MoveIt...");
    }

    void init()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper_group");
        
        arm_group_->setPlanningTime(5.0);          
        arm_group_->setNumPlanningAttempts(5); // 增加尝试次数

        sub_grasp_pose_ = this->create_subscription<my_robot_interfaces::msg::CamArmPose>(
            "/arm/grasp_pose", 10, std::bind(&ArmMoveItNode::grasp_pose_cb, this, _1));
            
        sub_gripper_cmd_ = this->create_subscription<my_robot_interfaces::msg::GripperState>(
            "/arm/grasp_status", 10, std::bind(&ArmMoveItNode::gripper_cmd_cb, this, _1));

        RCLCPP_INFO(this->get_logger(), "MoveIt 控制接口初始化完毕！等待 FSM 指令...");
    }

private:
    void grasp_pose_cb(const my_robot_interfaces::msg::CamArmPose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "--- 收到 FSM 抓取位姿目标 ---");

        geometry_msgs::msg::PointStamped pt_cam, pt_base;
        pt_cam.header.frame_id = "camera_link";
        pt_cam.header.stamp = this->get_clock()->now();
        pt_cam.point.x = msg->x; 
        pt_cam.point.y = msg->y; 
        pt_cam.point.z = msg->z;

        try {
            pt_base = tf_buffer_->transform(pt_cam, "arm_g_base", tf2::durationFromSec(1.0));
            RCLCPP_INFO(this->get_logger(), "TF 转换成功: x=%.2f, y=%.2f, z=%.2f", 
                        pt_base.point.x, pt_base.point.y, pt_base.point.z);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF 转换失败: %s", ex.what());
            return;
        }

        // ==========================================
        // 关键修改点：放宽约束，只要求到达位置点 (x, y, z)
        // ==========================================
        arm_group_->setPositionTarget(pt_base.point.x, pt_base.point.y, pt_base.point.z);

        // 3. 规划并执行
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "规划成功！正在执行...");
            arm_group_->execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "机械臂到达指定位置！");
        } else {
            RCLCPP_ERROR(this->get_logger(), "规划失败！可能原因：目标超出了机械臂的极限臂展。");
        }
    }

    void gripper_cmd_cb(const my_robot_interfaces::msg::GripperState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "--- 收到 FSM 夹爪控制指令 ---");
        std::string target_state = msg->grip ? "close" : "open";
        
        gripper_group_->setNamedTarget(target_state);
        if (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "夹爪动作 [%s] 执行成功！", target_state.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "夹爪动作执行失败！");
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    
    rclcpp::Subscription<my_robot_interfaces::msg::CamArmPose>::SharedPtr sub_grasp_pose_;
    rclcpp::Subscription<my_robot_interfaces::msg::GripperState>::SharedPtr sub_gripper_cmd_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMoveItNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    node->init();
    executor.spin();
    rclcpp::shutdown();
    return 0;
}