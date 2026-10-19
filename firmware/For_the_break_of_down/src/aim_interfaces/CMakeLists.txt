cmake_minimum_required(VERSION 3.8)
project(aim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 这句话是生成消息的核心！必须要有
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AimInfo.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
