cmake_minimum_required(VERSION 3.8)
project(armor_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aim_interfaces REQUIRED)

add_executable(armor_detector_node src/armor_detector_node.cpp)

# 告诉编译器去 include 文件夹里找头文件！
target_include_directories(armor_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(armor_detector_node PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  ${image_transport_TARGETS}
  ${OpenCV_LIBS}
  ${aim_interfaces_TARGETS} 
)

install(TARGETS armor_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
