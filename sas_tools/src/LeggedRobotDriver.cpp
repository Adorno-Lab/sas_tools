#include <sas_tools/LeggedRobotDriver.hpp>
#include <algorithm>
#include <stdexcept>

namespace sas
{

LeggedRobotDriver::~LeggedRobotDriver()
{

}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
LeggedRobotDriver::LeggedRobotDriver(std::atomic_bool* break_loops)
    : RobotDriver{break_loops}
{
}
#pragma GCC diagnostic pop


LeggedRobotDriver::LeggedRobotDriver(const std::shared_ptr<ShutdownSignaler> &shutdown_signaler)
    :RobotDriver{shutdown_signaler}
{

}

void LeggedRobotDriver::set_high_level_mode(const HIGH_LEVEL_MODE &mode)
{
    if (mode == current_mode_)
        return;

    const auto supported_modes = get_supported_high_level_modes();
    if (std::find(supported_modes.begin(), supported_modes.end(), mode) == supported_modes.end())
        throw std::invalid_argument("LeggedRobotDriver::set_high_level_mode: the mode " +
                                    std::to_string(static_cast<int>(mode)) + " is not supported.");

    _set_high_level_mode(mode);
    current_mode_ = mode;
}

LeggedRobotDriver::HIGH_LEVEL_MODE LeggedRobotDriver::get_high_level_mode() const
{
    return current_mode_;
}

LeggedRobotDriver::CommandAcceptance LeggedRobotDriver::get_command_acceptance() const
{
    const std::size_t number_of_limbs = get_limbs().size();

    CommandAcceptance acceptance;
    acceptance.limbs = std::vector<bool>(number_of_limbs, false);
    switch (current_mode_)
    {
    case HIGH_LEVEL_MODE::IDLE:
        return acceptance;
    case HIGH_LEVEL_MODE::STANDING:
        acceptance.base_orientation = is_supported(LEGGED_FUNCTIONALITY::BASE_ORIENTATION);
        acceptance.base_height      = is_supported(LEGGED_FUNCTIONALITY::BASE_HEIGHT);
        break;
    case HIGH_LEVEL_MODE::WALKING:
        acceptance.twist            = is_supported(LEGGED_FUNCTIONALITY::TWIST);
        acceptance.base_height      = is_supported(LEGGED_FUNCTIONALITY::BASE_HEIGHT);
        break;
    }

    acceptance.limbs = get_commandable_limbs();
    if (acceptance.limbs.size() != number_of_limbs)
        throw std::logic_error("LeggedRobotDriver::get_command_acceptance: get_commandable_limbs() returned " +
                               std::to_string(acceptance.limbs.size()) + " entries, but there are " +
                               std::to_string(number_of_limbs) + " limbs.");
    return acceptance;
}

VectorXd LeggedRobotDriver::get_joint_positions()
{
    return VectorXd();
}

void LeggedRobotDriver::set_target_joint_positions(const VectorXd&)
{

}

void LeggedRobotDriver::extra_control_loop_step()
{

}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> LeggedRobotDriver::get_base_orientation_limits() const
{
    return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}


}
