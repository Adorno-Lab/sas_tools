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
    enum class Mode{
        Idle=0,
        Stance,
        Walking
    };

protected:
    // Mode current_mode_{Mode::Idle};
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

    // Optional: Override these if needed, but can leave as default implementations
    // virtual VectorXd get_joint_velocities() override;
    // virtual VectorXd get_joint_torques() override;
    // virtual void set_target_joint_velocities(const VectorXd& set_target_joint_velocities) override;
    // virtual void set_target_joint_torques(const VectorXd& set_target_joint_torques) override;

    // New free-flying specific methods - PURE VIRTUAL
    virtual void set_twist(const DQ& twist) = 0;
    virtual DQ get_twist() = 0;

    // Mode management (can be overridden by concrete classes)
    //  void set_mode(const Mode& mode);
    //  Mode get_mode() const;
};

}
