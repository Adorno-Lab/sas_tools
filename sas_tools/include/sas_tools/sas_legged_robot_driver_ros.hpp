#pragma once

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_robot_driver/sas_robot_driver_server.hpp>
#include <sas_tools/LeggedRobotDriver.hpp>
#include <sas_tools/sas_legged_robot_driver_server.hpp>

namespace sas
{

struct LeggedRobotDriverROSConfiguration
{
    RobotDriverROSConfiguration robot_driver_ros;
    double twist_timeout_sec{0.2}; ///< A zero twist is sent when no twist arrived within this time.
};

/**
 * @brief The LeggedRobotDriverROS class runs a LeggedRobotDriver on the standard legged robot topics.
 *
 * It reuses RobotDriverROS (connect, initialize, timing, joint topics, watchdog, shutdown) and
 * installs the control loop callback of the driver, which in every iteration:
 *   1. applies the new high-level mode and passes the commands accepted in the current mode
 *      to the driver (see LeggedRobotDriver::get_command_acceptance()),
 *   2. publishes get/imu and get/status (and get/info in the first iteration),
 *   3. steps one RobotDriverServer per LeggedRobotDriver::get_manipulators() entry, under <prefix>/<name>,
 *   4. calls LeggedRobotDriver::extra_control_loop_step().
 *
 * Everything runs in the RobotDriverROS thread. Any exception ends the loop, and the whole robot stops.
 */
class LeggedRobotDriverROS
{
private:
    struct Manipulator
    {
        std::string name;
        std::shared_ptr<RobotDriver> driver;
        std::unique_ptr<RobotDriverServer> server;
    };

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<LeggedRobotDriver> legged_driver_;
    LeggedRobotDriverROSConfiguration configuration_;
    std::shared_ptr<ShutdownSignaler> shutdown_signaler_;
    std::string topic_prefix_;

    LeggedRobotDriverServer legged_server_;
    std::vector<Manipulator> manipulators_;
    RobotDriverROS robot_driver_ros_;
    bool info_sent_{false};

    void _control_loop_step();
    void _send_info();
    void _legged_step(const LeggedRobotDriver::CommandAcceptance& acceptance);
    void _manipulators_step(const LeggedRobotDriver::CommandAcceptance& acceptance);
    DQ _clamp_base_orientation(const DQ& r) const;

public:
    LeggedRobotDriverROS(const LeggedRobotDriverROS&)=delete;
    LeggedRobotDriverROS()=delete;

    /**
     * @brief LeggedRobotDriverROS
     * @param node The node on which every server is created.
     * @param legged_driver The driver. Its control loop callback is installed by control_loop().
     * @param configuration robot_driver_ros.robot_driver_provider_prefix is the topic prefix.
     * @param shutdown_signaler Shared with the driver and every RobotDriverROS loop.
     * @throws std::invalid_argument for a null driver, a non-positive twist timeout, or an invalid
     *         (empty, duplicated or null) manipulator entry.
     */
    LeggedRobotDriverROS(const std::shared_ptr<rclcpp::Node>& node,
                         const std::shared_ptr<LeggedRobotDriver>& legged_driver,
                         const LeggedRobotDriverROSConfiguration& configuration,
                         const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    /**
     * @brief control_loop Runs until shutdown. Always signals the shutdown before returning,
     *        so that the whole robot stops together.
     */
    int control_loop();
};

}
