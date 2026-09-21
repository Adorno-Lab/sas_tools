#pragma once

#include <sas_core/sas_robot_driver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <dqrobotics/DQ.h>

using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{

class LeggedRobotDriver: public RobotDriver
{
public:
    /**
     * @brief Enumeration of optional driver functionalities in high level mode
     */
    enum class HIGH_LEVEL_MODE{
        IDLE=0,
        STANDING,
        WALKING,
    };

protected:
    HIGH_LEVEL_MODE current_mode_{HIGH_LEVEL_MODE::IDLE};
    // Mode target_mode_{Mode::Idle};

public:
    LeggedRobotDriver(const LeggedRobotDriver&)=delete;
    LeggedRobotDriver()=delete;
    virtual ~LeggedRobotDriver();

    LeggedRobotDriver(std::atomic_bool* break_loops);
    LeggedRobotDriver(const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    // Required implementations from RobotDriver - PURE VIRTUAL
    virtual VectorXd get_joint_positions() override = 0;
    virtual void set_target_joint_positions(const VectorXd& set_target_joint_positions_rad) override = 0;
    virtual void connect() override = 0;
    virtual void disconnect() override = 0;
    virtual void initialize() override = 0;
    virtual void deinitialize() override = 0;

    /**
     * @brief set_target_twist Sets the desired twist of the robot's base.
     * @param twist Twist expressed at the body frame
     *
     * @note This command only takes effect when the robot is in the appropriate
     *       HIGH_LEVEL_MODE. The required mode depends on the specific robot
     *       implementation.
     */
    virtual void set_target_twist(const DQ& twist) = 0;

    /**
     * @brief set_target_base_orientation
     * @param r The unit quaternion that represents the target base orientation.
     * @note This command only takes effect when the robot is in the appropriate
     *       HIGH_LEVEL_MODE. The required mode depends on the specific robot
     *       implementation.
     */
    virtual void set_target_base_orientation(const DQ& r) = 0;

    /**
     * @brief set_target_base_height Sets the desired base height with respect to the ground.
     * @param height Target height in meters
     * @note This command only takes effect when the robot is in the appropriate
     *       HIGH_LEVEL_MODE. The required mode depends on the specific robot
     *       implementation.
     */
    virtual void set_target_base_height(const double& base_height) = 0;

    // --- IMU state (onboard IMU telemetry) ---
    // Unlike set_target_twist/set_target_base_orientation/set_target_base_height above,
    // these are state readings, not mode-gated commands: they reflect the robot's last
    // measured physical state as reported by its onboard IMU, and are expected to be
    // valid regardless of current_mode_ once the concrete driver is connected/
    // initialized (see each concrete implementation's own documentation for exactly
    // when a reading first becomes available). Left non-const, matching
    // get_joint_positions() above and the rest of this interface's existing
    // convention, even though a state getter would otherwise be a natural fit for
    // const.

    /**
     * @brief get_orientation Returns the robot base's last measured orientation, as
     *        reported by the onboard IMU.
     * @return A unit DQ quaternion (rotation only, no translation/dual part)
     *         representing the base orientation, in the same convention as the
     *         @p r accepted by set_target_base_orientation().
     */
    virtual DQ get_orientation() = 0;

    /**
     * @brief get_angular_velocity Returns the robot base's last measured angular
     *        velocity, as reported by the onboard IMU's gyroscope.
     * @return A 3-element VectorXd (x, y, z), in rad/s, expressed in the body frame.
     */
    virtual VectorXd get_angular_velocity() = 0;

    /**
     * @brief get_linear_acceleration Returns the robot base's last measured linear
     *        acceleration, as reported by the onboard IMU's accelerometer.
     * @return A 3-element VectorXd (x, y, z), in m/s^2, expressed in the body frame.
     *         This is the IMU's raw (proper/specific) acceleration reading -- it
     *         includes the gravity component, it is not a gravity-compensated
     *         estimate of the base's inertial acceleration.
     */
    virtual VectorXd get_linear_acceleration() = 0;
};

}
