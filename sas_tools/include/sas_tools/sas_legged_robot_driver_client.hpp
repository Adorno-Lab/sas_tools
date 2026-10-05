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
 * @brief The LeggedRobotDriverClient class is the client of the base of a LeggedRobotDriver served
 *        by LeggedRobotDriverROS. It extends RobotDriverClient with the legged robot topics, and uses
 *        the same MODE_BLACKLIST_FLAG:
 *   - JOINT_CONTROL also creates the legged command publishers (twist, mode, base height and
 *     orientation). Only one client per robot may use it.
 *   - JOINT_MONITORING also creates the legged state subscriptions (get/imu, get/status, get/info).
 *   - WATCHDOG_CONTROL is unchanged (set/watchdog_trigger). Only one client per robot may use it.
 *
 * Methods of a blacklisted mode throw std::runtime_error, as in RobotDriverClient.
 *
 * The base has no joints, so the joint getters inherited from RobotDriverClient are not used. The
 * joints belong to the limbs: read and command each limb with a RobotDriverClient on
 * \<prefix\>/\<name\>, using the limb names of get_limb_names().
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
    std::size_t _limb_index(const std::string& limb_name, const std::string& function) const;

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
     * @brief get_commandable_limbs Returns which limbs accept targets in the current mode, as reported
     *        on get/status.
     * @return One entry per limb of get_limb_names(), in the same order: true if the targets sent to
     *         \<prefix\>/\<name\>/set/target_joint_positions take effect. All false in IDLE.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/status was not received yet.
     */
    std::vector<bool> get_commandable_limbs() const;

    /**
     * @brief is_limb_commandable Returns true if the limb @p limb_name accepts targets in the current mode.
     * @param limb_name The name of the limb, e.g. "left_arm".
     * @return The entry of get_commandable_limbs() that corresponds to @p limb_name.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted, or get/info or get/status was not
     *         received yet.
     * @throws std::invalid_argument if the robot has no limb called @p limb_name.
     */
    bool is_limb_commandable(const std::string& limb_name) const;

    /**
     * @brief get_limb_names Returns the names of the limbs served by the driver, as reported on get/info.
     * @return The names, e.g. {"left_leg", "right_leg", "torso", "left_arm", "right_arm"} for the
     *         Unitree G1. Each limb is a standard SAS robot driver on \<prefix\>/\<name\>.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/info was not received yet.
     */
    std::vector<std::string> get_limb_names() const;

    /**
     * @brief get_limb_joint_names Returns the names of the joints of the limb @p limb_name, as reported
     *        on get/info.
     * @param limb_name The name of the limb, e.g. "left_arm".
     * @return One name per joint of \<prefix\>/\<name\>/get/joint_states, in the same order. Compare
     *         them with the kinematic model of the controller to check that it is connected to the
     *         robot it expects.
     * @throws std::runtime_error if JOINT_MONITORING is blacklisted or get/info was not received yet.
     * @throws std::invalid_argument if the robot has no limb called @p limb_name.
     */
    std::vector<std::string> get_limb_joint_names(const std::string& limb_name) const;

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
     * @brief is_enabled Returns true once the client received get/info and get/status. Wait for it,
     *        while spinning the node, before calling the state getters. The base has no joints, so
     *        the joint states of the base are not required; the limbs have their own clients.
     * @return Always false if JOINT_MONITORING is blacklisted.
     */
    bool is_enabled() const;
};

}
