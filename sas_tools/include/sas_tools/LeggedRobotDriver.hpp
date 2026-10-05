#pragma once

#include <memory>
#include <string>
#include <tuple>
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
 * A legged robot is modeled as a floating base with limbs. The legs, the arms, and the waist are all
 * limbs. A concrete driver only implements the virtual methods below; it does not use ROS.
 * LeggedRobotDriverROS exposes the driver on the standard legged robot topics and runs its control loop:
 *   - The base (twist, mode, IMU, status, and info) is served on \<prefix\>/.
 *   - Each limb of get_limbs() is served as a standard SAS robot driver on \<prefix\>/\<name\>.
 *
 * Every joint of the robot belongs to exactly one limb, and the base itself has no joints. Thus, a
 * client obtains every joint of any robot by concatenating the joints of its limbs.
 *
 * The high-level mode decides which commands take effect (the rules are applied by
 * LeggedRobotDriverROS, see get_command_acceptance()):
 *   - IDLE: the robot stays on and balancing; every target is ignored, the twist is zero,
 *     and the limbs hold still.
 *   - STANDING: the base orientation and the base height can be commanded; the robot cannot walk.
 *   - WALKING: the twist and the base height are accepted; the base orientation is not.
 *   - In STANDING and WALKING, the limbs accept targets as decided by get_commandable_limbs().
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
    enum class LEGGED_FUNCTIONALITY{
        TWIST=0,
        BASE_HEIGHT,
        BASE_ORIENTATION,
    };

    /**
     * @brief A limb (e.g. a leg, an arm, or the waist) served by this driver.
     *        LeggedRobotDriverROS exposes it with a standard RobotDriverServer
     *        under \<prefix\>/\<name\>.
     */
    struct LimbEntry
    {
        std::string name;                     ///< Topic suffix, e.g. "left_leg" or "left_arm".
        std::shared_ptr<RobotDriver> driver;  ///< The joints of the limb. Its lifecycle belongs to this driver.
        std::vector<std::string> joint_names; ///< One name per joint of the limb, in the order of its driver.
    };

    /**
     * @brief The commands that take effect in the current mode, given what the robot supports.
     */
    struct CommandAcceptance
    {
        bool twist{false};
        bool base_orientation{false};
        bool base_height{false};
        std::vector<bool> limbs;   ///< One entry per limb of get_limbs(): true if it accepts targets.
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
     *        after the standard base and limb steps. A generic client never depends on it.
     *        The default implementation does nothing.
     */
    virtual void extra_control_loop_step();

public:
    LeggedRobotDriver(const LeggedRobotDriver&)=delete;
    LeggedRobotDriver()=delete;
    virtual ~LeggedRobotDriver();

    LeggedRobotDriver(std::atomic_bool* break_loops);
    LeggedRobotDriver(const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    // --- Joints of the base ---

    /**
     * @brief get_joint_positions The base has no joints: every joint belongs to a limb of get_limbs().
     * @return An empty vector.
     */
    VectorXd get_joint_positions() final;

    /**
     * @brief set_target_joint_positions The base has no joints, so targets on \<prefix\>/set/target_joint_positions
     *        are ignored. Command the limbs on \<prefix\>/\<name\> instead.
     */
    void set_target_joint_positions(const VectorXd& set_target_joint_positions_rad) final;

    // --- Lifecycle (required). It also covers the drivers of every limb. ---
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
     * @note Only called in STANDING, if is_supported(LEGGED_FUNCTIONALITY::BASE_ORIENTATION), with the
     *       angles already clamped to get_base_orientation_limits().
     */
    virtual void set_target_base_orientation(const DQ& r) = 0;

    /**
     * @brief set_target_base_height Sets the desired base height with respect to the ground.
     * @param base_height Target height in meters
     * @note Only called in STANDING and WALKING, if is_supported(LEGGED_FUNCTIONALITY::BASE_HEIGHT).
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
     *        given is_supported() and get_commandable_limbs(). No limb accepts targets in IDLE.
     * @throws std::logic_error if get_commandable_limbs() does not have one entry per limb of get_limbs().
     */
    CommandAcceptance get_command_acceptance() const;

    // --- Description ---

    /**
     * @brief is_supported Returns true if the robot supports @p functionality.
     */
    virtual bool is_supported(const LEGGED_FUNCTIONALITY& functionality) const = 0;

    /**
     * @brief get_limbs Returns the limbs served by this driver, e.g. "left_leg", "right_leg", "waist",
     *        "left_arm", and "right_arm" for the Unitree G1, or the four legs for the Unitree B1 (whose
     *        Z1 arm runs its own driver). Every joint of the robot served by this driver must belong to
     *        exactly one limb. The list must not change during the lifetime of the driver:
     *        LeggedRobotDriverROS creates one server per limb at construction.
     * @return The limbs, in the order in which they are published in get/info.
     */
    virtual std::vector<LimbEntry> get_limbs() const = 0;

    /**
     * @brief get_commandable_limbs Tells which limbs apply the targets they receive on
     *        \<prefix\>/\<name\>/set/target_joint_positions in the current mode.
     *
     * The driver decides it per limb and per mode. For instance, in high-level control the Unitree G1
     * moves its legs with Unitree's own locomotion controller, so its legs never accept targets, while
     * its arms and waist accept them in STANDING. A robot that can move its arms while walking (e.g.
     * the Unitree H1) marks its arms as commandable in WALKING as well.
     *
     * LeggedRobotDriverROS only forwards the targets of a commandable limb, ignores every limb in IDLE,
     * and publishes the result in get/status (commandable_limbs).
     *
     * @return One entry per limb of get_limbs(), in the same order.
     */
    virtual std::vector<bool> get_commandable_limbs() const = 0;

    /**
     * @brief get_base_orientation_limits Returns the limits of set_target_base_orientation(), as ZYX
     *        roll-pitch-yaw angles in radians relative to F_f. The limits do not need to be symmetric.
     *        The default implementation returns zeros, which is correct only for robots that do not
     *        support LEGGED_FUNCTIONALITY::BASE_ORIENTATION.
     * @return {min, max}, each as {roll, pitch, yaw}, with min <= max for every angle.
     */
    virtual std::tuple<Eigen::Vector3d, Eigen::Vector3d> get_base_orientation_limits() const;

};

}
