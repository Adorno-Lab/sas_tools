/*
# (C) Copyright 2026 Adorno-Lab software developments
#
#    This file is part of sas_tools, distributed under the GNU Lesser General Public License v2.1 (see LICENSE).
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#
# ################################################################*/

#include "mainwindow.h"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

/**
 * Parameters (e.g. --ros-args -p legged_robot_prefix:=sas_b1/b1_2):
 *   - legged_robot_prefix (string, default "sas_b1/b1_1"): the legged robot driver.
 *   - own_driver_limb_prefixes (string array, default []): limbs with their own driver, e.g. "['sas_z1/z1_1']"
 *     for the Z1 arm of a B1.
 *   - mode (string, default "control"): "control" (the window commands the robot) or "monitor" (it only reads
 *     the robot, so that it can run next to the controller; the emergency stop and the watchdog still work).
 *   - max_vx, max_vy, max_wz (double, defaults 0.2 m/s, 0.1 m/s, 0.2 rad/s): ranges of the twist sliders.
 *   - watchdog_period_in_seconds (double, default 1.0), watchdog_maximum_acceptable_delay_in_seconds (double,
 *     default 0.5): sent with the watchdog triggers. The driver fixes them at the first trigger.
 */
int main(int argc, char *argv[])
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    // Keep our SIGINT handler: it closes the window, which stops the twist (and the watchdog).
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_legged_client_window");

    ClientWindowConfiguration configuration;
    configuration.legged_robot_prefix = node->declare_parameter<std::string>("legged_robot_prefix", "sas_b1/b1_1");
    configuration.own_driver_limb_prefixes = node->declare_parameter<std::vector<std::string>>("own_driver_limb_prefixes",
                                                                                               std::vector<std::string>{});
    const std::string mode = node->declare_parameter<std::string>("mode", "control");
    configuration.max_vx = node->declare_parameter<double>("max_vx", 0.2);
    configuration.max_vy = node->declare_parameter<double>("max_vy", 0.1);
    configuration.max_wz = node->declare_parameter<double>("max_wz", 0.2);
    configuration.watchdog_period_in_seconds = node->declare_parameter<double>("watchdog_period_in_seconds", 1.0);
    configuration.watchdog_maximum_acceptable_delay_in_seconds =
        node->declare_parameter<double>("watchdog_maximum_acceptable_delay_in_seconds", 0.5);

    if (mode == "control")
        configuration.mode = ClientWindowConfiguration::MODE::CONTROL;
    else if (mode == "monitor")
        configuration.mode = ClientWindowConfiguration::MODE::MONITOR;
    else
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "::The parameter mode must be \"control\" or \"monitor\", but it is \""
                                                << mode << "\".");
        rclcpp::shutdown();
        return 1;
    }
    if (!(configuration.watchdog_period_in_seconds > 0.0) || !(configuration.watchdog_maximum_acceptable_delay_in_seconds > 0.0))
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "::The watchdog period and maximum acceptable delay must be positive.");
        rclcpp::shutdown();
        return 1;
    }

    std::string own_driver_limb_prefixes;
    for (const auto& prefix : configuration.own_driver_limb_prefixes)
        own_driver_limb_prefixes += prefix + " ";
    RCLCPP_INFO_STREAM(node->get_logger(), "::legged_robot_prefix " << configuration.legged_robot_prefix
                                           << ", own_driver_limb_prefixes [ " << own_driver_limb_prefixes << "]"
                                           << ", mode " << mode
                                           << ", watchdog period " << configuration.watchdog_period_in_seconds << " s");

    int result = 0;
    {
        QApplication application(argc, argv);
        MainWindow window{node, configuration, &kill_this_process};
        window.show();
        result = application.exec();
    }
    rclcpp::shutdown();
    return result;
}
