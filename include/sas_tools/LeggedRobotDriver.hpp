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
    //  Mode target_mode_{Mode::Idle};

public:
    LeggedRobotDriver(const LeggedRobotDriver&)=delete;
    LeggedRobotDriver()=delete;

    ~LeggedRobotDriver();

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


};

}
