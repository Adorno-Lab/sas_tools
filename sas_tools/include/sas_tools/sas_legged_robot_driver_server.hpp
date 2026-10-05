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
 * @brief The LeggedRobotDriverServer class owns the legged robot topics under \<prefix\>/.
 *        It stores the latest commands and publishes the state it is given; it knows nothing
 *        about the robot. The joint topics are served by the standard RobotDriverServer.
 *
 *  Subscriptions (each accepts a single publisher; a second one makes the callback throw):
 *   - \<prefix\>/set/target_twist            geometry_msgs/TwistStamped (body frame)
 *   - \<prefix\>/set/high_level_mode         sas_legged_msgs/HighLevelMode
 *   - \<prefix\>/set/target_base_height      std_msgs/Float64 (meters)
 *   - \<prefix\>/set/target_base_orientation geometry_msgs/QuaternionStamped (relative to F_f)
 *  Publishers:
 *   - \<prefix\>/get/imu                     sensor_msgs/Imu
 *   - \<prefix\>/get/status                  sas_legged_msgs/LeggedRobotStatus
 *   - \<prefix\>/get/info                    sas_legged_msgs/LeggedRobotInfo (transient_local)
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
    /**
     * @brief LeggedRobotDriverServer Creates the legged robot topics under @p topic_prefix.
     * @param node The node on which the topics are created. The commands are only received while
     *        the node is spun (LeggedRobotDriverROS spins it in its control loop).
     * @param topic_prefix The prefix of the robot, e.g. "sas_g1/g1_1".
     */
    LeggedRobotDriverServer(const std::shared_ptr<rclcpp::Node>& node, const std::string& topic_prefix);

    // --- Commands received from the client ---

    /**
     * @brief has_received_target_twist Returns true once at least one twist was received on
     *        set/target_twist.
     * @return True if a twist was received.
     */
    bool has_received_target_twist() const;

    /**
     * @brief get_target_twist Returns the latest twist received on set/target_twist. Unlike the other
     *        commands, it is not consumed: the same twist is returned until a new one arrives. Use
     *        get_seconds_since_last_target_twist() to know how old it is.
     * @return The twist angular + E_*linear, expressed in the body frame. Zero if no twist was received.
     */
    DQ get_target_twist() const;

    /**
     * @brief get_seconds_since_last_target_twist Returns the time since the latest twist arrived on
     *        set/target_twist, measured with a steady clock (it is not affected by changes of the
     *        system clock or by the simulation time).
     * @return The elapsed time, in seconds. Infinity if no twist was received.
     */
    double get_seconds_since_last_target_twist() const;

    /**
     * @brief has_new_target_high_level_mode Returns true if a mode arrived on set/high_level_mode since
     *        the last call to get_target_high_level_mode().
     * @return True if a new mode is waiting.
     */
    bool has_new_target_high_level_mode() const;

    /**
     * @brief get_target_high_level_mode Returns the latest mode received on set/high_level_mode, and
     *        clears the flag of has_new_target_high_level_mode(). Invalid mode values are rejected
     *        when they arrive, so the returned mode is always one of HIGH_LEVEL_MODE.
     * @return The requested mode. IDLE if no mode was received.
     */
    LeggedRobotDriver::HIGH_LEVEL_MODE get_target_high_level_mode();

    /**
     * @brief has_new_target_base_height Returns true if a base height arrived on set/target_base_height
     *        since the last call to get_target_base_height().
     * @return True if a new base height is waiting.
     */
    bool has_new_target_base_height() const;

    /**
     * @brief get_target_base_height Returns the latest base height received on set/target_base_height,
     *        and clears the flag of has_new_target_base_height().
     * @return The target height of the base with respect to the ground, in meters. Zero if no height
     *         was received.
     */
    double get_target_base_height();

    /**
     * @brief has_new_target_base_orientation Returns true if a base orientation arrived on
     *        set/target_base_orientation since the last call to get_target_base_orientation().
     * @return True if a new base orientation is waiting.
     */
    bool has_new_target_base_orientation() const;

    /**
     * @brief get_target_base_orientation Returns the latest base orientation received on
     *        set/target_base_orientation, and clears the flag of has_new_target_base_orientation().
     * @return A unit quaternion relative to F_f (the received quaternion is normalized; zero
     *         quaternions are rejected when they arrive). DQ(1) if no orientation was received.
     */
    DQ get_target_base_orientation();

    // --- State sent to the clients ---

    /**
     * @brief send_imu Publishes the IMU state on get/imu.
     * @param orientation Unit quaternion of the base orientation, in the IMU's own world frame.
     * @param angular_velocity Pure quaternion, in rad/s, expressed in the body frame.
     * @param linear_acceleration Pure quaternion, in m/s^2, expressed in the body frame (with gravity).
     */
    void send_imu(const DQ& orientation, const DQ& angular_velocity, const DQ& linear_acceleration);

    /**
     * @brief send_status Publishes the current state of the driver on get/status.
     * @param mode The current mode.
     * @param acceptance The commands that take effect in the current mode, including which limbs
     *        accept targets (one entry per limb of get/info, in the same order).
     */
    void send_status(const LeggedRobotDriver::HIGH_LEVEL_MODE& mode,
                     const LeggedRobotDriver::CommandAcceptance& acceptance);

    /**
     * @brief send_info Publishes the static description of the robot on get/info. The topic is
     *        transient_local, so clients that connect later also receive the last description.
     * @param info The description: limbs and their joint names, supported modes and functionalities,
     *        and base orientation limits.
     */
    void send_info(const sas_legged_msgs::msg::LeggedRobotInfo& info);
};

}
