#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <sas_core/sas_robot_driver.hpp>
#include <dqrobotics/DQ.h>

using namespace DQ_robotics;

namespace sas
{

class LeggedRobotDriverROS;

/**
 * @brief The LeggedRobotDriver class is the hardware interface of a legged robot.
 *
 * A concrete driver only implements the virtual methods below; it does not use ROS.
 * LeggedRobotDriverROS exposes the driver on the standard legged robot topics and runs
 * its control loop.
 *
 * The high-level mode decides which commands take effect (the rules are applied by
 * LeggedRobotDriverROS, see get_command_acceptance()):
 *   - IDLE: the robot stays on and balancing; every target is ignored, the twist is zero,
 *     and the arms hold still.
 *   - STANDING: the arms, the base orientation and the base height can be commanded; the
 *     robot cannot walk.
 *   - WALKING: the twist and the base height are accepted; the base orientation is not.
 *     The arms are accepted only if is_supported(LeggedFunctionality::ManipulationWhileWalking).
 *
 * The driver starts in IDLE.
 */
class LeggedRobotDriver: public RobotDriver
{
public:
    /**
     * @brief Enumeration of the high-level modes.
     */
    enum class HIGH_LEVEL_MODE{
        IDLE=0,
        STANDING,
        WALKING,
    };

    /**
     * @brief Enumeration of the optional legged robot functionalities.
     */
    enum class LeggedFunctionality{
        Twist=0,
        BaseHeight,
        BaseOrientation,
        ManipulationWhileWalking,
    };

    /**
     * @brief A manipulator (e.g. an arm or the waist) served by this driver.
     *        LeggedRobotDriverROS exposes it with a standard RobotDriverServer
     *        under <prefix>/<name>.
     */
    struct ManipulatorEntry
    {
        std::string name;
        std::shared_ptr<RobotDriver> driver;
    };

    /**
     * @brief The commands that take effect in the current mode, given what the robot supports.
     */
    struct CommandAcceptance
    {
        bool twist{false};
        bool base_orientation{false};
        bool base_height{false};
        bool manipulators{false};
    };

private:
    friend class LeggedRobotDriverROS;
    // The control loop callback belongs to LeggedRobotDriverROS. Concrete drivers must not install their own.
    using RobotDriver::set_control_loop_callback;

protected:
    HIGH_LEVEL_MODE current_mode_{HIGH_LEVEL_MODE::IDLE};

    /**
     * @brief _set_high_level_mode Performs the robot-specific mode change (e.g. stopping before
     *        switching from WALKING to STANDING). Called by set_high_level_mode() only with a
     *        supported mode that differs from the current one; current_mode_ is updated after it returns.
     * @param mode The new mode.
     */
    virtual void _set_high_level_mode(const HIGH_LEVEL_MODE& mode) = 0;

    /**
     * @brief extra_control_loop_step Robot-specific work executed once per control loop iteration,
     *        after the standard legged and manipulator steps. A generic client never depends on it.
     *        The default implementation does nothing.
     */
    virtual void extra_control_loop_step();

public:
    LeggedRobotDriver(const LeggedRobotDriver&)=delete;
    LeggedRobotDriver()=delete;
    virtual ~LeggedRobotDriver();

    LeggedRobotDriver(std::atomic_bool* break_loops);
    LeggedRobotDriver(const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    // Required implementations from RobotDriver - PURE VIRTUAL
    virtual VectorXd get_joint_positions() override = 0;

    /**
     * @brief set_target_joint_positions Sets the target positions of every joint of get_joint_positions().
     * @param set_target_joint_positions_rad The full joint vector, in radians. The driver must ignore
     *        the entries where get_commandable_joint_mask() is false (all of them in IDLE).
     */
    virtual void set_target_joint_positions(const VectorXd& set_target_joint_positions_rad) override = 0;
    virtual void connect() override = 0;
    virtual void disconnect() override = 0;
    virtual void initialize() override = 0;
    virtual void deinitialize() override = 0;

    /**
     * @brief set_target_twist Sets the desired twist of the robot's base.
     * @param twist Twist expressed at the body frame (angular + E_*linear).
     *
     * @note LeggedRobotDriverROS sends a zero twist in every iteration in which the current mode
     *       does not accept a twist, or when no twist arrived within the timeout. The driver must
     *       accept a zero twist in every mode.
     */
    virtual void set_target_twist(const DQ& twist) = 0;

    /**
     * @brief set_target_base_orientation Sets the desired base orientation.
     * @param r The unit quaternion r = r_z(yaw)*r_y(pitch)*r_x(roll) (ZYX roll-pitch-yaw angles),
     *        relative to the frame F_f. F_f is the base frame at the moment the robot entered
     *        STANDING, so r = 1 keeps the pose the robot had when STANDING started.
     * @note Only called in STANDING, if is_supported(LeggedFunctionality::BaseOrientation), with the
     *       angles already clamped to get_base_orientation_limits().
     */
    virtual void set_target_base_orientation(const DQ& r) = 0;

    /**
     * @brief set_target_base_height Sets the desired base height with respect to the ground.
     * @param base_height Target height in meters
     * @note Only called in STANDING and WALKING, if is_supported(LeggedFunctionality::BaseHeight).
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
     *         representing the base orientation in the IMU's own world frame (its yaw
     *         can drift). This is not the frame F_f of set_target_base_orientation().
     *         Return DQ(0) while no reading is available yet.
     */
    virtual DQ get_orientation() = 0;

    /**
     * @brief get_angular_velocity Returns the robot base's last measured angular
     *        velocity, as reported by the onboard IMU's gyroscope.
     * @return A pure quaternion (wx*i_ + wy*j_+ wz*k_), in rad/s, expressed in the body frame.
     */
    virtual DQ get_angular_velocity() = 0;

    /**
     * @brief get_linear_acceleration Returns the robot base's last measured linear
     *        acceleration, as reported by the onboard IMU's accelerometer.
     * @return A pure quaternion (a*i_ + a*j_+ a*k_), in m/s^2, expressed in the body frame.
     *         This is the IMU's raw (proper/specific) acceleration reading -- it
     *         includes the gravity component, it is not a gravity-compensated
     *         estimate of the base's inertial acceleration.
     */
    virtual DQ get_linear_acceleration() = 0;

    // --- High-level mode ---

    /**
     * @brief set_high_level_mode Changes the high-level mode. Does nothing if @p mode is the current mode.
     * @param mode The new mode.
     * @throws std::invalid_argument if the robot does not support @p mode.
     */
    void set_high_level_mode(const HIGH_LEVEL_MODE& mode);

    /**
     * @brief get_high_level_mode Returns the current high-level mode.
     */
    HIGH_LEVEL_MODE get_high_level_mode() const;

    /**
     * @brief get_supported_high_level_modes Returns the modes the robot supports. Must include IDLE.
     */
    virtual std::vector<HIGH_LEVEL_MODE> get_supported_high_level_modes() const = 0;

    /**
     * @brief get_command_acceptance Returns the commands that take effect in the current mode,
     *        given is_supported() and get_manipulators().
     */
    CommandAcceptance get_command_acceptance() const;

    // --- Description ---

    /**
     * @brief get_robot_model Returns the robot model, e.g. "Unitree G1".
     */
    virtual std::string get_robot_model() const = 0;

    /**
     * @brief is_supported Returns true if the robot supports @p functionality.
     */
    virtual bool is_supported(const LeggedFunctionality& functionality) const = 0;

    /**
     * @brief get_joint_names Returns the names of every joint of get_joint_positions(), in the same order.
     */
    virtual std::vector<std::string> get_joint_names() const = 0;

    /**
     * @brief get_commandable_joint_mask Returns, for every joint of get_joint_positions(), whether
     *        set_target_joint_positions() commands it in the current mode. All false in IDLE.
     */
    virtual std::vector<bool> get_commandable_joint_mask() const = 0;

    /**
     * @brief get_base_orientation_limits Returns the symmetric limits {roll, pitch, yaw}, in radians, of
     *        set_target_base_orientation(). The default implementation returns zeros, which is correct
     *        only for robots that do not support LeggedFunctionality::BaseOrientation.
     */
    virtual std::array<double, 3> get_base_orientation_limits() const;

    /**
     * @brief get_manipulators Returns the manipulators served by this driver. The default
     *        implementation returns an empty list (e.g. B1, whose Z1 arm runs its own driver).
     */
    virtual std::vector<ManipulatorEntry> get_manipulators() const;
};

}
