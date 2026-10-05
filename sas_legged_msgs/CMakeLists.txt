cmake_minimum_required(VERSION 3.8)
project(sas_legged_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/HighLevelMode.msg"
    "msg/LimbInfo.msg"
    "msg/LeggedRobotStatus.msg"
    "msg/LeggedRobotInfo.msg"
    DEPENDENCIES
    std_msgs
    )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
