#include "sas_robot_driver_unitree_b1/LeggedRobotDriver.hpp"

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
