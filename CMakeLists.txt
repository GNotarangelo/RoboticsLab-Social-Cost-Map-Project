cmake_minimum_required(VERSION 3.8)
project(social_cost_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Mutuated from fra2mo
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)

#Mutuated from fra2mo
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(people_msgs REQUIRED)

add_executable(dynamic_tf_publisher src/odom_bl_tf.cpp)
add_executable(social_costmap_bridge src/social_costmap_bridge.cpp)
add_executable(fra2mo_navigation src/fra2mo_navigation.cpp)
add_executable(iiwa_cube_routine src/iiwa_cube_routine.cpp)
ament_target_dependencies(dynamic_tf_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros)
ament_target_dependencies(social_costmap_bridge rclcpp tf2_msgs people_msgs)
ament_target_dependencies(fra2mo_navigation rclcpp rclcpp_action nav2_msgs geometry_msgs people_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(iiwa_cube_routine rclcpp moveit_ros_planning_interface moveit_core aruco_opencv_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  worlds
  urdf
  srdf
  meshes
  models
  launch
  src
  moveit2
  config
  rviz_conf
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
 dynamic_tf_publisher
 DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
 social_costmap_bridge
 DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  fra2mo_navigation
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  iiwa_cube_routine
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
