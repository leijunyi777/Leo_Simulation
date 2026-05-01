cmake_minimum_required(VERSION 3.8)
project(mycobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# 生成可执行文件
add_executable(arm_moveit_node src/arm_moveit_node.cpp)

# 链接依赖
ament_target_dependencies(arm_moveit_node
  rclcpp
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  my_robot_interfaces
)

# 配置安装路径
install(TARGETS
  arm_moveit_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()