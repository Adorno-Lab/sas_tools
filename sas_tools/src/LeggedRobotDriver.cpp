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
    CommandAcceptance acceptance;
    switch (current_mode_)
    {
    case HIGH_LEVEL_MODE::IDLE:
        break;
    case HIGH_LEVEL_MODE::STANDING:
        acceptance.base_orientation = is_supported(LeggedFunctionality::BaseOrientation);
        acceptance.base_height      = is_supported(LeggedFunctionality::BaseHeight);
        acceptance.manipulators     = true;
        break;
    case HIGH_LEVEL_MODE::WALKING:
        acceptance.twist            = is_supported(LeggedFunctionality::Twist);
        acceptance.base_height      = is_supported(LeggedFunctionality::BaseHeight);
        acceptance.manipulators     = is_supported(LeggedFunctionality::ManipulationWhileWalking);
        break;
    }
    acceptance.manipulators = acceptance.manipulators && !get_manipulators().empty();
    return acceptance;
}

void LeggedRobotDriver::extra_control_loop_step()
{

}

std::array<double, 3> LeggedRobotDriver::get_base_orientation_limits() const
{
    return {0.0, 0.0, 0.0};
}

std::vector<LeggedRobotDriver::ManipulatorEntry> LeggedRobotDriver::get_manipulators() const
{
    return {};
}

}
