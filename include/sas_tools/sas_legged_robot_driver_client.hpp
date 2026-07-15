#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>

using namespace rclcpp;

namespace sas
{
class LeggedRobotDriverClient: public RobotDriverClient
{
public:
    LeggedRobotDriverClient();
};
}
