#include <sas_tools/LeggedRobotDriver.hpp>

namespace sas
{

LeggedRobotDriver::~LeggedRobotDriver()
{

}

LeggedRobotDriver::LeggedRobotDriver(
    std::atomic_bool* break_loops)
:RobotDriver{break_loops}
{


}

LeggedRobotDriver::LeggedRobotDriver(const std::shared_ptr<ShutdownSignaler> &shutdown_signaler)
    :RobotDriver{shutdown_signaler}
{

}


}
