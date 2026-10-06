#include <sas_tools/sas_legged_robot_driver_ros.hpp>
#include <sas_tools/rpy_conversions.hpp>
#include <set>
#include <stdexcept>

namespace sas
{

namespace
{

template<typename T>
const std::shared_ptr<T>& _check_not_null(const std::shared_ptr<T>& pointer, const std::string& name)
{
    if (!pointer)
        throw std::invalid_argument("LeggedRobotDriverROS: " + name + " is null.");
    return pointer;
}

const LeggedRobotDriverROSConfiguration& _check_configuration(const LeggedRobotDriverROSConfiguration& configuration)
{
    if (configuration.robot_driver_ros.robot_driver_provider_prefix.empty())
        throw std::invalid_argument("LeggedRobotDriverROS: robot_driver_ros.robot_driver_provider_prefix is empty.");
    if (!(configuration.twist_timeout_sec > 0.0))
        throw std::invalid_argument("LeggedRobotDriverROS: twist_timeout_sec must be positive.");
    return configuration;
}

}

LeggedRobotDriverROS::LeggedRobotDriverROS(const std::shared_ptr<rclcpp::Node> &node,
                                           const std::shared_ptr<LeggedRobotDriver> &legged_driver,
                                           const LeggedRobotDriverROSConfiguration &configuration,
                                           const std::shared_ptr<ShutdownSignaler> &shutdown_signaler):
    node_(_check_not_null(node, "node")),
    legged_driver_(_check_not_null(legged_driver, "legged_driver")),
    configuration_(_check_configuration(configuration)),
    shutdown_signaler_(_check_not_null(shutdown_signaler, "shutdown_signaler")),
    topic_prefix_(configuration.robot_driver_ros.robot_driver_provider_prefix),
    legged_server_(node_, topic_prefix_),
    robot_driver_ros_(node_, legged_driver_, configuration_.robot_driver_ros, shutdown_signaler_)
{
    std::set<std::string> names;
    for (const auto& entry : legged_driver_->get_limbs())
    {
        if (entry.name.empty() || !entry.driver)
            throw std::invalid_argument("LeggedRobotDriverROS: every limb needs a name and a driver.");
        if (!names.insert(entry.name).second)
            throw std::invalid_argument("LeggedRobotDriverROS: the limb name " + entry.name + " is duplicated.");

        limbs_.push_back({entry.name,
                                 entry.driver,
                                 std::make_unique<RobotDriverServer>(node_, topic_prefix_ + "/" + entry.name)});
        RCLCPP_INFO_STREAM(node_->get_logger(), "::Serving the limb " << entry.name
                                                << " on " << topic_prefix_ << "/" << entry.name);
    }
}

int LeggedRobotDriverROS::control_loop()
{
    legged_driver_->set_control_loop_callback([this]() { _control_loop_step(); });
    const int result = robot_driver_ros_.control_loop();
    // RobotDriverROS returns on shutdown and on any exception. Signal the shutdown in both
    // cases, so that every loop sharing shutdown_signaler_ stops with this one.
    shutdown_signaler_->shutdown();
    return result;
}

void LeggedRobotDriverROS::_control_loop_step()
{
    if (!info_sent_)
    {
        _send_info();
        info_sent_ = true;
    }

    if (legged_server_.has_new_target_high_level_mode())
    {
        const auto mode = legged_server_.get_target_high_level_mode();
        if (mode != legged_driver_->get_high_level_mode())
        {
            try
            {
                legged_driver_->set_high_level_mode(mode);
                RCLCPP_INFO_STREAM(node_->get_logger(), "::High-level mode changed to " << static_cast<int>(mode));
            }
            catch (const std::invalid_argument& e)
            {
                RCLCPP_WARN_STREAM(node_->get_logger(), "::Mode change rejected: " << e.what());
            }
        }
    }

    const auto acceptance = legged_driver_->get_command_acceptance();
    _legged_step(acceptance);
    _limbs_step(acceptance);
    legged_driver_->extra_control_loop_step();
}

void LeggedRobotDriverROS::_send_info()
{
    sas_legged_msgs::msg::LeggedRobotInfo info;
    for (const auto& entry : legged_driver_->get_limbs())
    {
        sas_legged_msgs::msg::LimbInfo limb;
        limb.name = entry.name;
        limb.joint_names = entry.joint_names;
        info.limbs.push_back(limb);
    }
    for (const auto& mode : legged_driver_->get_supported_high_level_modes())
        info.supported_modes.push_back(static_cast<uint8_t>(mode));

    using F = LeggedRobotDriver::LEGGED_FUNCTIONALITY;
    info.supports_twist = legged_driver_->is_supported(F::TWIST);
    info.supports_base_height = legged_driver_->is_supported(F::BASE_HEIGHT);
    info.supports_base_orientation = legged_driver_->is_supported(F::BASE_ORIENTATION);

    const auto [min_rpy, max_rpy] = _get_base_orientation_limits();
    info.min_base_roll  = min_rpy(0);
    info.max_base_roll  = max_rpy(0);
    info.min_base_pitch = min_rpy(1);
    info.max_base_pitch = max_rpy(1);
    info.min_base_yaw   = min_rpy(2);
    info.max_base_yaw   = max_rpy(2);

    legged_server_.send_info(info);
}

void LeggedRobotDriverROS::_legged_step(const LeggedRobotDriver::CommandAcceptance &acceptance)
{
    if (legged_driver_->is_supported(LeggedRobotDriver::LEGGED_FUNCTIONALITY::TWIST))
    {
        const bool twist_is_fresh = legged_server_.has_received_target_twist() &&
                                    legged_server_.get_seconds_since_last_target_twist() <= configuration_.twist_timeout_sec;
        legged_driver_->set_target_twist(acceptance.twist && twist_is_fresh ? legged_server_.get_target_twist() : DQ(0));
    }

    // New targets are consumed even when ignored, so that an old target is never applied after a mode change.
    if (legged_server_.has_new_target_base_height())
    {
        const double base_height = legged_server_.get_target_base_height();
        if (acceptance.base_height)
            legged_driver_->set_target_base_height(base_height);
    }
    if (legged_server_.has_new_target_base_orientation())
    {
        const DQ r = legged_server_.get_target_base_orientation();
        if (acceptance.base_orientation)
            legged_driver_->set_target_base_orientation(_clamp_base_orientation(r));
    }

    const DQ orientation = legged_driver_->get_orientation();
    if (is_quaternion(orientation) && is_unit(orientation))
        legged_server_.send_imu(orientation,
                                legged_driver_->get_angular_velocity(),
                                legged_driver_->get_linear_acceleration());

    legged_server_.send_status(legged_driver_->get_high_level_mode(), acceptance);
}

void LeggedRobotDriverROS::_limbs_step(const LeggedRobotDriver::CommandAcceptance &acceptance)
{
    for (std::size_t i = 0; i < limbs_.size(); i++)
    {
        auto& limb = limbs_.at(i);
        auto& server = *limb.server;
        auto& driver = limb.driver;

        if (server.get_shutdown_signal())
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::The shutdown signal was received on "
                                                         << topic_prefix_ << "/" << limb.name << "!");
            shutdown_signaler_->shutdown();
        }

        // Same forwarding as RobotDriverROS. Velocity and force control are optional for a driver.
        if (acceptance.limbs.at(i))
        {
            if (server.is_enabled())
                driver->set_target_joint_positions(server.get_target_joint_positions());
            if (server.is_enabled(RobotDriver::Functionality::VelocityControl))
            {
                try{driver->set_target_joint_velocities(server.get_target_joint_velocities());} catch(...){}
            }
            if (server.is_enabled(RobotDriver::Functionality::ForceControl))
            {
                try{driver->set_target_joint_torques(server.get_target_joint_forces());} catch(...){}
            }
        }

        const VectorXd joint_positions = driver->get_joint_positions();
        VectorXd joint_velocities;
        try{joint_velocities = driver->get_joint_velocities();} catch(...){}
        VectorXd joint_torques;
        try{joint_torques = driver->get_joint_torques();} catch(...){}

        server.send_joint_states(joint_positions, joint_velocities, joint_torques);
        server.send_joint_limits(driver->get_joint_limits());
    }
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> LeggedRobotDriverROS::_get_base_orientation_limits() const
{
    const auto limits = legged_driver_->get_base_orientation_limits();
    if ((std::get<0>(limits).array() > std::get<1>(limits).array()).any())
        throw std::invalid_argument("LeggedRobotDriverROS: get_base_orientation_limits() returned min > max.");
    return limits;
}

DQ LeggedRobotDriverROS::_clamp_base_orientation(const DQ &r) const
{
    const auto [min_rpy, max_rpy] = _get_base_orientation_limits();
    const Eigen::Vector3d rpy = unit_quaternion_to_rpy(r);
    const Eigen::Vector3d clamped = rpy.cwiseMax(min_rpy).cwiseMin(max_rpy);

    if (clamped != rpy)
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                    "::The target base orientation was clamped to the limits of the robot.");
    return rpy_to_unit_quaternion(clamped(0), clamped(1), clamped(2));
}

}
