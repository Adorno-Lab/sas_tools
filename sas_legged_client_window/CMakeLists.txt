cmake_minimum_required(VERSION 3.16)
project(sas_legged_client_window LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Qt 6 of the system (e.g. sudo apt install qt6-base-dev).
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_robot_driver REQUIRED)
find_package(sas_tools REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    Eigen3
    sas_core
    sas_robot_driver
    sas_tools
)

target_link_libraries(${PROJECT_NAME}
    Qt6::Core
    Qt6::Widgets
    dqrobotics
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

ament_package()
