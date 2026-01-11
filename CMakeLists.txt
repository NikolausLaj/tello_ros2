cmake_minimum_required(VERSION 3.8)
project(tello_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Include directories for C++ headers
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Add executable for C++ node
add_executable(tello_rect_images src/tello_rect_image_node.cpp)
ament_target_dependencies(tello_rect_images
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(tello_rect_images ${OpenCV_LIBRARIES})

# Add executable for tello_terrain_follow_node
add_executable(tello_terrain_follow src/tello_terrain_follow_node.cpp)
ament_target_dependencies(tello_terrain_follow
  rclcpp
  sensor_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS tello_rect_images
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tello_terrain_follow
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/tello_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  tello_ros2/teleop_twist_rpyt_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs OpenCV)

ament_package()