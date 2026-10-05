#pragma once

#include <string>
#include <tuple>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sas_legged_msgs/msg/high_level_mode.hpp>
#include <sas_legged_msgs/msg/legged_robot_status.hpp>
#include <sas_legged_msgs/msg/legged_robot_info.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_tools/LeggedRobotDriver.hpp>

namespace sas
{

/**
 * @brief The LeggedRobotDriverClient class is the client of a LeggedRobotDriver served by
 *        LeggedRobotDriverROS. It extends RobotDriverClient (joint topics) with the legged
 *        robot topics, and uses the same MODE_BLACKLIST_FLAG:
 *   - JOINT_CONTROL also creates the legged command publishers (twist, mode, base height and
 *     orientation). Only one client per robot may use it.
 *   - JOINT_MONITORING also creates the legged state subscriptions (get/imu, get/status, get/info).
 *   - WATCHDOG_CONTROL is unchanged (set/watchdog_trigger). Only one client per robot may use it.
 *
 * Methods of a blacklisted mode throw std::runtime_error, as in RobotDriverClient.
 */
class LeggedRobotDriverClient: public RobotDriverClient
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string topic_prefix_;
    std::vector<MODE_BLACKLIST_FLAG> blacklisted_modes_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_target_twist_;
    rclcpp::Publisher<sas_legged_msgs::msg::HighLevelMode>::SharedPtr publisher_high_level_mode_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_target_base_height_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr publisher_target_base_orientation_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
    DQ orientation_{0};
    DQ angular_velocity_{0};
    DQ linear_acceleration_{0};
    bool imu_received_{false};

    rclcpp::Subscription<sas_legged_msgs::msg::LeggedRobotStatus>::SharedPtr subscriber_status_;
    sas_legged_msgs::msg::LeggedRobotStatus status_;
    bool status_received_{false};

    rclcpp::Subscription<sas_legged_msgs::msg::LeggedRobotInfo>::SharedPtr subscriber_info_;
    sas_legged_msgs::msg::LeggedRobotInfo info_;
    bool info_received_{false};

    bool _is_blacklisted(const MODE_BLACKLIST_FLAG& mode) const;
    void _check_not_blacklisted(const MODE_BLACKLIST_FLAG& mode, const std::string& function) const;
    void _check_exclusive_publisher(const std::string& topic) const;
    void _check_received(const bool& received, const std::string& topic, const std::string& function) const;

    void _callback_imu(const sensor_msgs::msg::Imu& msg);
    void _callback_status(const sas_legged_msgs::msg::LeggedRobotStatus& msg);
    void _callback_info(const sas_legged_msgs::msg::LeggedRobotInfo& msg);

public:
    LeggedRobotDriverClient() = delete;
    LeggedRobotDriverClient(const LeggedRobotDriverClient&) = delete;

    /**
     * @brief LeggedRobotDriverClient
     * @param node The node on which the topics are created.
     * @param topic_prefix The prefix of the robot, e.g. "sas_g1/g1_1".
     * @param blacklisted_modes The modes this client does not use (e.g. {WATCHDOG_CONTROL} for a
     *        kinematic controller, {JOINT_CONTROL} for a watchdog node).
     */
    LeggedRobotDriverClient(const std::shared_ptr<rclcpp::Node>& node,
                            const std::string& topic_prefix,
                            const std::vector<MODE_BLACKLIST_FLAG>& blacklisted_modes = std::vector<MODE_BLACKLIST_FLAG>{});

    // --- Commands (JOINT_CONTROL) ---

    /**
     * @brief send_target_twist Sends the target twist of the base (angular + E_*linear, body frame).
     *        Only takes effect in a mode where accepts_twist() is true.
     */
    void send_target_twist(const DQ& twist);

    void send_high_level_mode(const LeggedRobotDriver::HIGH_LEVEL_MODE& mode);

    /**
     * @brief send_target_base_height Sends the target base height, in meters.
     *        Only takes effect in a mode where accepts_base_height() is true.
     */
    void send_target_base_height(const double& base_height);

    /**
     * @brief send_target_base_orientation Sends the target base orientation.
     * @param r Unit quaternion relative to the frame F_f (the base frame when STANDING started).
     *        Only takes effect in a mode where accepts_base_orientation() is true.
     * @throws std::invalid_argument if @p r is not a unit quaternion.
     */
    void send_target_base_orientation(const DQ& r);

    /**
     * @brief send_target_base_orientation_rpy Sends the target base orientation as ZYX
     *        roll-pitch-yaw angles, in radians, relative to the frame F_f.
     */
    void send_target_base_orientation_rpy(const double& roll, const double& pitch, const double& yaw);

    // --- State (JOINT_MONITORING) ---

    DQ get_orientation() const;
    DQ get_angular_velocity() const;
    DQ get_linear_acceleration() const;

    LeggedRobotDriver::HIGH_LEVEL_MODE get_high_level_mode() const;
    std::vector<bool> get_commandable_joint_mask() const;
    bool accepts_twist() const;
    bool accepts_base_orientation() const;
    bool accepts_base_height() const;
    bool accepts_manipulator_commands() const;

    std::vector<std::string> get_joint_names() const;
    std::vector<LeggedRobotDriver::HIGH_LEVEL_MODE> get_supported_high_level_modes() const;
    bool is_supported(const LeggedRobotDriver::LEGGED_FUNCTIONALITY& functionality) const;

    /**
     * @brief get_base_orientation_limits Returns the limits of the base orientation, as ZYX
     *        roll-pitch-yaw angles in radians relative to F_f.
     * @return {min, max}, each as {roll, pitch, yaw}.
     */
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> get_base_orientation_limits() const;

    /**
     * @brief is_enabled Returns true once the joint states (see RobotDriverClient::is_enabled()),
     *        get/info and get/status were received.
     */
    bool is_enabled(const RobotDriver::Functionality& supported_functionality=RobotDriver::Functionality::PositionControl) const;
};

}
