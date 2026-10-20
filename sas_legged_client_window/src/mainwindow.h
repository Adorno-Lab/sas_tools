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

#pragma once
#include <QMainWindow>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sas_msgs/msg/bool.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_tools/sas_legged_robot_driver_client.hpp>

class QAbstractButton;
class QButtonGroup;
class QCheckBox;
class QGridLayout;
class QGroupBox;
class QLabel;
class QPushButton;
class QRadioButton;
class QSlider;

struct ClientWindowConfiguration
{
    /**
     * @brief CONTROL: the window commands the robot (mode, twist, base pose, limbs). MONITOR: it only reads the
     *        robot, so that it can run next to the controller of the robot (the command topics are not created).
     *        The emergency stop and the watchdog are available in both modes.
     */
    enum class MODE{CONTROL, MONITOR};

    std::string legged_robot_prefix;                    ///< The legged robot driver, e.g. "sas_b1/b1_1".
    std::vector<std::string> own_driver_limb_prefixes;  ///< Limbs with their own driver, e.g. {"sas_z1/z1_1"}.
    MODE mode{MODE::CONTROL};
    double max_vx{0.2};                                 ///< Range of the twist sliders: [-max_vx, max_vx] m/s, ...
    double max_vy{0.1};                                 ///< ... [-max_vy, max_vy] m/s, ...
    double max_wz{0.2};                                 ///< ... and [-max_wz, max_wz] rad/s.
    double watchdog_period_in_seconds{1.0};             ///< Initial position of the period slider (rounded to 1-5 s).
    double watchdog_maximum_acceptable_delay_in_seconds{1e30};
    // The emergency stop also publishes on this topic (e.g. the one of the emergency stop of the lab, to which the
    // launch files of the drivers remap their set/shutdown). Empty: only the set/shutdown of each driver.
    std::string shutdown_topic{"/sas/set/shutdown"};
};

/**
 * @brief The MainWindow class tests a legged robot driver of sas_tools (e.g. the Unitree B1) through
 *        LeggedRobotDriverClient, and one RobotDriverClient per limb.
 *
 * The limbs of the legged driver are built from get/info (their names, joints, and number of joints), so
 * the window does not depend on the robot. The limbs with their own driver (e.g. the Z1 arm) are built
 * once their joint states arrive. The legged driver decides when its limbs accept targets (get/status);
 * a limb with its own driver does not know the mode of the base, so it only accepts targets from this
 * window while "Allow commands" is checked.
 *
 * The twist of the sliders is sent on every timer tick while the robot accepts it, since the driver stops
 * the robot when no twist arrives within its twist timeout.
 *
 * The watchdog (WATCHDOG_CONTROL) is started on request, for the legged driver and for every limb with its own
 * driver. Its triggers are sent by their own thread, so that a busy window does not trip it. It cannot be stopped:
 * when the window closes, the triggers stop, and the drivers stop the robot.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(const std::shared_ptr<rclcpp::Node>& node,
               const ClientWindowConfiguration& configuration,
               std::atomic_bool* break_loops,
               QWidget *parent = nullptr);
    ~MainWindow();

private:
    struct JointWidgets
    {
        QLabel* position{nullptr};
        QLabel* velocity{nullptr};
        QLabel* torque{nullptr};
        QSlider* target{nullptr};
        QLabel* target_value{nullptr};
    };

    struct LimbWidgets
    {
        std::string name;                                  ///< Limb name (legged driver) or topic prefix (own driver).
        bool own_driver{false};                            ///< Served by its own driver, not by the legged driver.
        std::shared_ptr<sas::RobotDriverClient> client;
        std::vector<std::string> joint_names;              ///< From get/info (legged driver), or generated (own driver).
        QGroupBox* group{nullptr};                         ///< Built once the number of joints is known.
        QLabel* status{nullptr};
        QCheckBox* allow_commands{nullptr};                ///< Own driver only.
        std::vector<JointWidgets> joints;
        bool limits_configured{false};
    };

    std::shared_ptr<rclcpp::Node> node_;
    ClientWindowConfiguration configuration_;
    std::atomic_bool* break_loops_;
    bool monitor_only_{false};
    std::vector<sas::RobotDriverClient::MODE_BLACKLIST_FLAG> blacklist_;
    std::shared_ptr<sas::LeggedRobotDriverClient> legged_client_;
    std::vector<LimbWidgets> limbs_;
    bool legged_limbs_created_{false};

    const double slider_factor_ = 1000.0; // Sliders are integers: milli-(m/s, rad/s) and millirad.

    int timer_id_{0};
    const int time_step_in_milliseconds_{50};  // Well below the twist timeout of the driver (0.2 s).
    double elapsed_time_{0};

    //---------- Widgets ---------------------------------------------------------//
    QLabel* label_connection_{nullptr};
    QLabel* label_mode_{nullptr};
    QLabel* label_capabilities_{nullptr};
    QLabel* label_elapsed_time_{nullptr};

    QButtonGroup* button_group_mode_{nullptr};
    QRadioButton* radio_idle_{nullptr};
    QRadioButton* radio_standing_{nullptr};
    QRadioButton* radio_walking_{nullptr};

    QGroupBox* group_twist_{nullptr};
    QSlider* slider_vx_{nullptr};
    QSlider* slider_vy_{nullptr};
    QSlider* slider_wz_{nullptr};
    QLabel* label_vx_{nullptr};
    QLabel* label_vy_{nullptr};
    QLabel* label_wz_{nullptr};
    bool accepted_twist_{false};

    QGroupBox* group_base_pose_{nullptr};
    QSlider* slider_roll_{nullptr};
    QSlider* slider_pitch_{nullptr};
    QSlider* slider_yaw_{nullptr};
    QSlider* slider_height_{nullptr};
    QLabel* label_roll_{nullptr};
    QLabel* label_pitch_{nullptr};
    QLabel* label_yaw_{nullptr};
    QLabel* label_height_{nullptr};
    bool accepted_base_pose_{false};
    bool base_orientation_limits_configured_{false};
    bool base_height_limits_configured_{false};
    double nominal_base_height_{0.0};                    ///< From get/info: the position of the height slider in the neutral pose.

    QLabel* label_quaternion_{nullptr};
    QLabel* label_rpy_{nullptr};
    QLabel* label_gyroscope_{nullptr};
    QLabel* label_accelerometer_{nullptr};
    QLabel* label_measured_twist_{nullptr};

    QGridLayout* layout_limbs_{nullptr};
    QPushButton* button_emergency_stop_{nullptr};
    rclcpp::Publisher<sas_msgs::msg::Bool>::SharedPtr publisher_shutdown_;

    //---------- Watchdog --------------------------------------------------------//
    QPushButton* button_watchdog_{nullptr};
    QLabel* label_watchdog_{nullptr};
    // The period of the watchdog, chosen with a slider (1 to 5 s) before it starts. The driver fixes it at the first
    // trigger, so the slider is disabled once the watchdog starts.
    static constexpr int min_watchdog_period_in_seconds_ = 1;
    static constexpr int max_watchdog_period_in_seconds_ = 5;
    QSlider* slider_watchdog_period_{nullptr};
    QLabel* label_watchdog_period_{nullptr};
    double watchdog_period_in_seconds_{1.0};
    // One client per driver (the legged driver and each limb with its own driver), with only WATCHDOG_CONTROL.
    // Created when the watchdog starts, and not changed afterwards, so the thread can use them without a lock.
    std::vector<std::shared_ptr<sas::RobotDriverClient>> watchdog_clients_;
    std::thread watchdog_thread_;
    std::atomic_bool watchdog_running_{false};
    void _on_start_watchdog_clicked();
    void _watchdog_loop();

    void _build_ui();
    QGroupBox* _build_status_group();
    QGroupBox* _build_mode_group();
    QGroupBox* _build_twist_group();
    QGroupBox* _build_imu_group();
    QGroupBox* _build_base_pose_group();
    QSlider* _add_twist_row(QGridLayout* layout, const int& row, const QString& name, const double& max_value,
                            QLabel*& value_label);

    //---------- Update (timer) --------------------------------------------------//
    void timerEvent(QTimerEvent *event) override;
    void _update();
    void _update_status();
    void _update_mode();
    void _update_imu();
    void _create_legged_limbs();
    void _update_limbs();
    void _update_elapsed_time();

    //---------- Limbs -----------------------------------------------------------//
    void _build_limb_group(LimbWidgets& limb);
    bool _is_commandable(const LimbWidgets& limb) const;
    void _on_limb_slider_moved(const std::size_t& limb_index);

    //---------- Commands --------------------------------------------------------//
    void _on_mode_clicked(QAbstractButton* button);
    void _on_twist_slider_changed();
    void _send_target_twist();
    void _zero_twist();
    void _on_base_orientation_slider_changed();
    void _on_base_height_slider_changed();
    void _neutral_base_pose();
    void _on_emergency_stop_clicked();

    void _show_error(const std::string& what);
    static QString _format(const double& value, const int& decimals = 3);
};
