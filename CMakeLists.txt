cmake_minimum_required(VERSION 3.8)
project(self_driving_cpp)

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Add compile options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  src/Detection/Lanes
)

# Add executables
add_executable(self_driving_node src/self_driving_node.cpp)
add_executable(spawning_node src/spawner_node.cpp)
add_executable(video_recorder src/video_recorder.cpp)
add_executable(computer_vision_node 
                src/computer_vision_node.cpp 
                src/Detection/Lanes/color_segmentation.cpp 
                src/Detection/Lanes/lane_detection.cpp
                src/Detection/Lanes/morphology.cpp
                src/drive_bot.cpp
                config/config.cpp)

# Link dependencies for each executable
ament_target_dependencies(self_driving_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(spawning_node
  rclcpp
  gazebo_msgs
)

ament_target_dependencies(video_recorder
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(computer_vision_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  geometry_msgs
  Eigen3
)

# Install targets
install(TARGETS
  self_driving_node
  spawning_node
  video_recorder
  computer_vision_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources like launch files, worlds, and scripts
install(DIRECTORY launch worlds scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install specific scripts with execute permission
install(PROGRAMS
  scripts/lights_spawner.bash
  scripts/lights_spawner_maze.bash
  DESTINATION lib/${PROJECT_NAME}/
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()











# cmake_minimum_required(VERSION 3.8)
# project(self_driving_cpp)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(gazebo_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)

# # include_directories(
# #   include/self_driving_cpp/color_segmentation.hpp
# # )
# # Add executables

# # Include directories
# include_directories(
#   include
#   include/self_driving_cpp
#   src/Detection/Lanes
# )


# add_executable(self_driving_node src/self_driving_node.cpp)
# add_executable(spawning_node src/spawner_node.cpp)
# add_executable(video_recorder src/video_recorder.cpp)
# add_executable(computer_vision_node 
#                 src/computer_vision_node.cpp 
#                 src/Detection/Lanes/color_segmentation.cpp 
#                 src/Detection/Lanes/lane_detection.cpp
#                 src/Detection/Lanes/morphology.cpp
#                 config/config.cpp
#                 src/drive_bot.cpp)

# ament_target_dependencies(computer_vision_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   OpenCV
#   geometry_msgs
#   Eigen3
# )

# # Link dependencies
# ament_target_dependencies(self_driving_node
#   rclcpp
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
# )

# ament_target_dependencies(spawning_node
#   rclcpp
#   gazebo_msgs
# )

# ament_target_dependencies(video_recorder
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   OpenCV
# )

# # Install targets
# install(TARGETS
#   self_driving_node
#   spawning_node
#   video_recorder
#   computer_vision_node
#   DESTINATION lib/${PROJECT_NAME})

# # Install other resources like launch files
# install(DIRECTORY launch worlds scripts
#   DESTINATION share/${PROJECT_NAME}/
# )

# # Install scripts
# install(PROGRAMS
#   scripts/lights_spawner.bash
#   scripts/lights_spawner_maze.bash
#   DESTINATION lib/${PROJECT_NAME}/
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
