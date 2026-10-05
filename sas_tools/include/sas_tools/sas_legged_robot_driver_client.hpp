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
     * @brief LeggedRobotDriverClient Creates the client of the robot served under @p topic_prefix.
     * @param node The node on which the topics are created. The node must be spun so that the
     *        state getters receive data.
     * @param topic_prefix The prefix of the robot, e.g. "sas_g1/g1_1".
     * @param blacklisted_modes The modes this client does not use, e.g. {WATCHDOG_CONTROL} for a
     *        kinematic controller, or {JOINT_CONTROL} for a watchdog node. The topics of a
     *        blacklisted mode are not created.
     */
    LeggedRobotDriverClient(const std::shared_ptr<rclcpp::Node>& node,
                            const std::string& topic_prefix,
                            const std::vector<MODE_BLACKLIST_FLAG>& blacklisted_modes = std::vector<MODE_BLACKLIST_FLAG>{});

    // --- Commands (JOINT_CONTROL) ---

    /**
     * @brief send_target_twist Sends the target twist of the base on set/target_twist.
     * @param twist The twist angular + E_*linear, expressed in the body frame (angular in rad/s,
     *        linear in m/s). It only takes effect in a mode where accepts_twist() is true; if no
     *        twist arrives within the twist timeout of the driver, the robot stops.
     * @throws std::runtime_error if JOINT_CONTROL is blacklisted.
     */
    void send_target_twist(const DQ& twist);

    /**
     * @brief send_high_level_mode Requests a high-level mode on set/high_level_mode.
     * @param mode The requested mode. The driver ignores a mode it does not support; read
     *        get_high_level_mode() to know whether the change happened.
     * @throws std::runtime_error if JOINT_CONTROL is blacklisted.
     */
    void send_high_level_mode(const LeggedRobotDriver::HIGH_LEVEL_MODE& mode);

    /**
     * @brief send_target_base_height Sends the target base height on set/target_base_height.
     * @param base_height The height of the base with respect to the ground, in meters. It only
     *        takes effect in a mode where accepts_base_height() is true.
     * @throws std::runtime_error if JOINT_CONTROL is blacklisted.
     */
    void send_target_base_height(const double& base_height);

    /**
     * @brief send_target_base_orientation Sends the target base orientation on set/target_base_orientation.
     * @param r Unit quaternion relative to the frame F_f (the base frame when STANDING started). It
     *        only takes effect in a mode where accepts_base_orientation() is true, and the driver
     *        clamps it to get_base_orientation_limits().
     * @throws std::runtime_error if JOINT_CONTROL is blacklisted.
     * @throws std::invalid_argument if @p r is not a unit quaternion.
     */
    void send_target_base_orientation(const DQ& r);

    /**
     * @brief send_target_base_orientation_rpy Sends the target base orientation as ZYX roll-pitch-yaw
     *        angles, i.e., r = r_z(yaw)*r_y(pitch)*r_x(roll), relative to the frame F_f.
     * @param roll Rotation around the x-axis, in radians.
     * @param pitch Rotation around the y-axis, in radians.
     * @param yaw Rotation around the z-axis, in radians.
     * @throws std::runtime_error if JOINT_CONTROL is blacklisted.
     */
    void send_target_base_orientation_rpy(const double& roll, const double& pitch, const double& yaw);

    // --- State (JOINT_MONITORING) ---

    /**
     * @brief get_orientation Returns the latest base orientation reported on get/imu.
     * @return A unit quaternion in the IMU's own world frame (its yaw can drift). This is not the
     *         frame F_f of send_target_base_orientation().
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/imu was not received yet.
     */
    DQ get_orientation() const;

    /**
     * @brief get_angular_velocity Returns the latest base angular velocity reported on get/imu.
     * @return A pure quaternion (wx*i_ + wy*j_ + wz*k_), in rad/s, expressed in the body frame.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/imu was not received yet.
     */
    DQ get_angular_velocity() const;

    /**
     * @brief get_linear_acceleration Returns the latest base linear acceleration reported on get/imu.
     * @return A pure quaternion (ax*i_ + ay*j_ + az*k_), in m/s^2, expressed in the body frame. It is
     *         the raw accelerometer reading, so it includes gravity.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/imu was not received yet.
     */
    DQ get_linear_acceleration() const;

    /**
     * @brief get_high_level_mode Returns the current mode of the robot, as reported on get/status.
     * @return The mode the driver is in. It may differ from the last mode sent with
     *         send_high_level_mode() if the driver rejected it.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/status was not received yet.
     */
    LeggedRobotDriver::HIGH_LEVEL_MODE get_high_level_mode() const;

    /**
     * @brief get_commandable_joint_mask Returns which joints take set/target_joint_positions in the
     *        current mode, as reported on get/status.
     * @return One entry per joint of get_joint_positions(), in the order of get_joint_names(). The
     *         driver ignores the targets of the joints whose entry is false.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/status was not received yet.
     */
    std::vector<bool> get_commandable_joint_mask() const;

    /**
     * @brief accepts_twist Returns true if send_target_twist() takes effect in the current mode.
     * @return The accepts_twist field of the latest get/status.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/status was not received yet.
     */
    bool accepts_twist() const;

    /**
     * @brief accepts_base_orientation Returns true if send_target_base_orientation() takes effect in
     *        the current mode.
     * @return The accepts_base_orientation field of the latest get/status.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/status was not received yet.
     */
    bool accepts_base_orientation() const;

    /**
     * @brief accepts_base_height Returns true if send_target_base_height() takes effect in the current mode.
     * @return The accepts_base_height field of the latest get/status.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/status was not received yet.
     */
    bool accepts_base_height() const;

    /**
     * @brief accepts_manipulator_commands Returns true if the manipulators served by this driver
     *        (under \<prefix\>/\<name\>) take their joint targets in the current mode.
     * @return The accepts_manipulator_commands field of the latest get/status.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/status was not received yet.
     */
    bool accepts_manipulator_commands() const;

    /**
     * @brief get_joint_names Returns the names of the joints of get_joint_positions(), as reported on get/info.
     * @return One name per joint, in the same order as get_joint_positions(). Compare them with the
     *         kinematic model of the controller to check that it is connected to the robot it expects.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/info was not received yet.
     */
    std::vector<std::string> get_joint_names() const;

    /**
     * @brief get_supported_high_level_modes Returns the modes the robot supports, as reported on get/info.
     * @return The supported modes. IDLE is always among them.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/info was not received yet.
     */
    std::vector<LeggedRobotDriver::HIGH_LEVEL_MODE> get_supported_high_level_modes() const;

    /**
     * @brief is_supported Returns true if the robot supports @p functionality, as reported on get/info.
     * @param functionality The functionality to check.
     * @return True if the robot supports @p functionality.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/info was not received yet.
     */
    bool is_supported(const LeggedRobotDriver::LEGGED_FUNCTIONALITY& functionality) const;

    /**
     * @brief get_base_orientation_limits Returns the limits of the base orientation, as reported on get/info.
     * @return {min, max}, each as ZYX {roll, pitch, yaw} angles in radians relative to F_f. Zero if
     *         the robot does not support LEGGED_FUNCTIONALITY::BASE_ORIENTATION.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/info was not received yet.
     */
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> get_base_orientation_limits() const;

    /**
     * @brief is_enabled Returns true once the client received everything it needs: the joint states
     *        (see RobotDriverClient::is_enabled()), get/info and get/status. Wait for it, while
     *        spinning the node, before calling the state getters.
     * @param supported_functionality The joint functionality to check, as in RobotDriverClient::is_enabled().
     * @return Always false if JOINT_MONITORING is blacklisted.
     */
    bool is_enabled(const RobotDriver::Functionality& supported_functionality=RobotDriver::Functionality::PositionControl) const;
};

}
