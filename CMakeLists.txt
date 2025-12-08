cmake_minimum_required(VERSION 3.8)
project(tello_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories for C++ headers
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable for C++ node
add_executable(tello_rect_images src/tello_rect_image_node.cpp)
ament_target_dependencies(tello_rect_images
  rclcpp
  sensor_msgs
)
target_link_libraries(tello_rect_images ${OpenCV_LIBRARIES})

ament_python_install_package(${PROJECT_NAME})

install(TARGETS tello_rect_images
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/tello_bridge_node
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