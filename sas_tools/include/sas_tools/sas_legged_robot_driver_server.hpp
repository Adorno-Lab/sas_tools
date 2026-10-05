#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sas_legged_msgs/msg/high_level_mode.hpp>
#include <sas_legged_msgs/msg/legged_robot_status.hpp>
#include <sas_legged_msgs/msg/legged_robot_info.hpp>
#include <sas_tools/LeggedRobotDriver.hpp>

namespace sas
{

/**
 * @brief The LeggedRobotDriverServer class owns the legged robot topics under <prefix>/.
 *        It stores the latest commands and publishes the state it is given; it knows nothing
 *        about the robot. The joint topics are served by the standard RobotDriverServer.
 *
 *  Subscriptions (each accepts a single publisher; a second one makes the callback throw):
 *   - <prefix>/set/target_twist            geometry_msgs/TwistStamped (body frame)
 *   - <prefix>/set/high_level_mode         sas_legged_msgs/HighLevelMode
 *   - <prefix>/set/target_base_height      std_msgs/Float64 (meters)
 *   - <prefix>/set/target_base_orientation geometry_msgs/QuaternionStamped (relative to F_f)
 *  Publishers:
 *   - <prefix>/get/imu                     sensor_msgs/Imu
 *   - <prefix>/get/status                  sas_legged_msgs/LeggedRobotStatus
 *   - <prefix>/get/info                    sas_legged_msgs/LeggedRobotInfo (transient_local)
 */
class LeggedRobotDriverServer
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string topic_prefix_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_target_twist_;
    DQ target_twist_{0};
    bool target_twist_received_{false};
    std::chrono::steady_clock::time_point target_twist_time_point_;

    rclcpp::Subscription<sas_legged_msgs::msg::HighLevelMode>::SharedPtr subscriber_high_level_mode_;
    LeggedRobotDriver::HIGH_LEVEL_MODE target_high_level_mode_{LeggedRobotDriver::HIGH_LEVEL_MODE::IDLE};
    bool new_target_high_level_mode_{false};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_target_base_height_;
    double target_base_height_{0.0};
    bool new_target_base_height_{false};

    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subscriber_target_base_orientation_;
    DQ target_base_orientation_{1};
    bool new_target_base_orientation_{false};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
    rclcpp::Publisher<sas_legged_msgs::msg::LeggedRobotStatus>::SharedPtr publisher_status_;
    rclcpp::Publisher<sas_legged_msgs::msg::LeggedRobotInfo>::SharedPtr publisher_info_;

    void _check_exclusive_publisher(const std::string& topic) const;
    void _callback_target_twist(const geometry_msgs::msg::TwistStamped& msg);
    void _callback_high_level_mode(const sas_legged_msgs::msg::HighLevelMode& msg);
    void _callback_target_base_height(const std_msgs::msg::Float64& msg);
    void _callback_target_base_orientation(const geometry_msgs::msg::QuaternionStamped& msg);

public:
    LeggedRobotDriverServer() = delete;
    LeggedRobotDriverServer(const LeggedRobotDriverServer&) = delete;
    LeggedRobotDriverServer(const std::shared_ptr<rclcpp::Node>& node, const std::string& topic_prefix);

    /**
     * @brief has_received_target_twist Returns true once at least one twist was received.
     */
    bool has_received_target_twist() const;

    /**
     * @brief get_target_twist Returns the latest twist (angular + E_*linear, body frame).
     */
    DQ get_target_twist() const;

    /**
     * @brief get_seconds_since_last_target_twist Returns the time since the latest twist arrived,
     *        measured with a steady clock. Infinity if no twist was received.
     */
    double get_seconds_since_last_target_twist() const;

    /**
     * @brief has_new_target_high_level_mode Returns true if a mode arrived since the last
     *        call to get_target_high_level_mode().
     */
    bool has_new_target_high_level_mode() const;
    LeggedRobotDriver::HIGH_LEVEL_MODE get_target_high_level_mode();

    /**
     * @brief has_new_target_base_height Returns true if a base height arrived since the last
     *        call to get_target_base_height().
     */
    bool has_new_target_base_height() const;
    double get_target_base_height();

    /**
     * @brief has_new_target_base_orientation Returns true if a base orientation arrived since the
     *        last call to get_target_base_orientation().
     */
    bool has_new_target_base_orientation() const;

    /**
     * @brief get_target_base_orientation Returns the latest target base orientation (unit quaternion,
     *        relative to F_f).
     */
    DQ get_target_base_orientation();

    /**
     * @brief send_imu Publishes get/imu.
     * @param orientation Unit quaternion of the base orientation.
     * @param angular_velocity Pure quaternion, in rad/s, body frame.
     * @param linear_acceleration Pure quaternion, in m/s^2, body frame (with gravity).
     */
    void send_imu(const DQ& orientation, const DQ& angular_velocity, const DQ& linear_acceleration);

    /**
     * @brief send_status Publishes get/status.
     */
    void send_status(const LeggedRobotDriver::HIGH_LEVEL_MODE& mode,
                     const std::vector<bool>& commandable_joints,
                     const LeggedRobotDriver::CommandAcceptance& acceptance);

    /**
     * @brief send_info Publishes get/info. Late subscribers also receive it (transient_local).
     */
    void send_info(const sas_legged_msgs::msg::LeggedRobotInfo& info);
};

}
