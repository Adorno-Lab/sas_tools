#include <sas_tools/LeggedRobotDriver.hpp>

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


}
