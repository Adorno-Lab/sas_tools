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
#include <QButtonGroup>
#include <QCheckBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSlider>
#include <QStatusBar>
#include <QVBoxLayout>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sas_tools/rpy_conversions.hpp>

using HIGH_LEVEL_MODE = sas::LeggedRobotDriver::HIGH_LEVEL_MODE;
using MODE_BLACKLIST_FLAG = sas::RobotDriverClient::MODE_BLACKLIST_FLAG;

MainWindow::MainWindow(const std::shared_ptr<rclcpp::Node> &node,
                       const ClientWindowConfiguration &configuration,
                       std::atomic_bool *break_loops,
                       QWidget *parent)
    : QMainWindow(parent),
    node_{node},
    configuration_{configuration},
    break_loops_{break_loops},
    monitor_only_{configuration.mode == ClientWindowConfiguration::MODE::MONITOR},
    watchdog_period_in_seconds_{std::clamp(std::round(configuration.watchdog_period_in_seconds),
                                           static_cast<double>(min_watchdog_period_in_seconds_),
                                           static_cast<double>(max_watchdog_period_in_seconds_))}
{
    // The watchdog has its own clients (see _on_start_watchdog_clicked()). In monitor mode, the command topics are
    // not created either, so this window can run next to the controller of the robot.
    blacklist_ = {MODE_BLACKLIST_FLAG::WATCHDOG_CONTROL};
    if (monitor_only_)
        blacklist_.push_back(MODE_BLACKLIST_FLAG::JOINT_CONTROL);

    legged_client_ = std::make_shared<sas::LeggedRobotDriverClient>(node_, configuration_.legged_robot_prefix, blacklist_);
    if (!configuration_.shutdown_topic.empty())
        publisher_shutdown_ = node_->create_publisher<sas_msgs::msg::Bool>(configuration_.shutdown_topic, 1);

    // The limbs with their own driver are known from the configuration; their widgets are built once their
    // joint states arrive. The limbs of the legged driver are created from get/info.
    for (const auto& prefix : configuration_.own_driver_limb_prefixes)
    {
        LimbWidgets limb;
        limb.name = prefix;
        limb.own_driver = true;
        limb.client = std::make_shared<sas::RobotDriverClient>(node_, prefix, blacklist_);
        limbs_.push_back(std::move(limb));
    }

    _build_ui();
    timer_id_ = startTimer(time_step_in_milliseconds_);
}

MainWindow::~MainWindow()
{
    if (timer_id_ != 0)
        killTimer(timer_id_);
    // The triggers stop: the drivers stop the robot when the watchdog period elapses.
    watchdog_running_ = false;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    // Stop walking now, instead of waiting for the twist timeout of the driver.
    try
    {
        if (!monitor_only_ && legged_client_->is_enabled() && legged_client_->accepts_twist())
            legged_client_->send_target_twist(DQ(0));
    }
    catch (const std::exception& e)
    {
        std::cerr<<"MainWindow::~MainWindow. "<<e.what()<<std::endl;
    }
}

//---------------------------------------------------------------------------------//
//------------------------------------ UI -----------------------------------------//
//---------------------------------------------------------------------------------//

void MainWindow::_build_ui()
{
    setWindowTitle(QString("Legged robot client: ") + configuration_.legged_robot_prefix.c_str() +
                   (monitor_only_ ? " (monitor)" : " (control)"));

    auto central = new QWidget(this);
    auto layout_main = new QVBoxLayout(central);

    // Status, mode, and emergency stop
    auto layout_top = new QHBoxLayout();
    layout_top->addWidget(_build_status_group(), 2);
    layout_top->addWidget(_build_mode_group(), 1);
    button_emergency_stop_ = new QPushButton("EMERGENCY\nSTOP", central);
    button_emergency_stop_->setObjectName("button_emergency_stop");
    button_emergency_stop_->setToolTip(QString("Sends the shutdown signal to the legged driver, to every limb with its own "
                                               "driver, and on %1.").arg(configuration_.shutdown_topic.empty() ?
                                                                         "no other topic" : configuration_.shutdown_topic.c_str()));
    button_emergency_stop_->setMinimumSize(160, 90);
    button_emergency_stop_->setStyleSheet("QPushButton { background-color: #c62828; color: white; font-weight: bold; font-size: 16px; }");
    connect(button_emergency_stop_, &QPushButton::clicked, this, [this]() { _on_emergency_stop_clicked(); });

    auto layout_watchdog = new QVBoxLayout();
    button_watchdog_ = new QPushButton("START\nWATCHDOG", central);
    button_watchdog_->setObjectName("button_start_watchdog");
    button_watchdog_->setToolTip("Sends watchdog triggers to the legged driver and to every limb with its own driver. "
                                 "It cannot be stopped: when this window closes, the drivers stop the robot.");
    button_watchdog_->setMinimumSize(160, 60);
    connect(button_watchdog_, &QPushButton::clicked, this, [this]() { _on_start_watchdog_clicked(); });
    label_watchdog_ = new QLabel("Watchdog: off", central);
    label_watchdog_->setAlignment(Qt::AlignCenter);

    auto layout_watchdog_period = new QHBoxLayout();
    slider_watchdog_period_ = new QSlider(Qt::Horizontal, central);
    slider_watchdog_period_->setObjectName("slider_watchdog_period");
    slider_watchdog_period_->setToolTip("Period of the watchdog. The driver fixes it at the first trigger, so it cannot "
                                        "be changed once the watchdog starts.");
    slider_watchdog_period_->setRange(min_watchdog_period_in_seconds_, max_watchdog_period_in_seconds_);
    slider_watchdog_period_->setSingleStep(1);
    slider_watchdog_period_->setPageStep(1);
    slider_watchdog_period_->setTickInterval(1);
    slider_watchdog_period_->setTickPosition(QSlider::TicksBelow);
    slider_watchdog_period_->setValue(static_cast<int>(watchdog_period_in_seconds_));
    label_watchdog_period_ = new QLabel(QString("Period: %1 s").arg(watchdog_period_in_seconds_), central);
    connect(slider_watchdog_period_, &QSlider::valueChanged, this, [this](int value) {
        watchdog_period_in_seconds_ = value;
        label_watchdog_period_->setText(QString("Period: %1 s").arg(value));
    });
    layout_watchdog_period->addWidget(label_watchdog_period_);
    layout_watchdog_period->addWidget(slider_watchdog_period_);

    layout_watchdog->addWidget(button_watchdog_);
    layout_watchdog->addLayout(layout_watchdog_period);
    layout_watchdog->addWidget(label_watchdog_);
    layout_top->addLayout(layout_watchdog);
    layout_top->addWidget(button_emergency_stop_);
    layout_main->addLayout(layout_top);

    // Twist and IMU
    auto layout_middle = new QHBoxLayout();
    layout_middle->addWidget(_build_twist_group(), 1);
    layout_middle->addWidget(_build_base_pose_group(), 1);
    layout_middle->addWidget(_build_imu_group(), 1);
    layout_main->addLayout(layout_middle);

    // Limbs, built at runtime
    auto group_limbs = new QGroupBox("Limbs", central);
    auto layout_group_limbs = new QVBoxLayout(group_limbs);
    auto scroll_area = new QScrollArea(group_limbs);
    scroll_area->setWidgetResizable(true);
    auto limbs_widget = new QWidget(scroll_area);
    layout_limbs_ = new QGridLayout(limbs_widget);
    layout_limbs_->setAlignment(Qt::AlignTop);
    scroll_area->setWidget(limbs_widget);
    layout_group_limbs->addWidget(scroll_area);
    layout_main->addWidget(group_limbs, 1);

    setCentralWidget(central);
    resize(1500, 900);
    statusBar()->showMessage("Waiting for " + QString(configuration_.legged_robot_prefix.c_str()) + "/get/info and get/status...");
}

QGroupBox *MainWindow::_build_status_group()
{
    auto group = new QGroupBox("Status", this);
    auto layout = new QGridLayout(group);
    label_connection_ = new QLabel(group);
    label_mode_ = new QLabel(group);
    label_capabilities_ = new QLabel(group);
    label_elapsed_time_ = new QLabel("00:00:00", group);
    label_capabilities_->setWordWrap(true);
    layout->addWidget(new QLabel("Robot:", group), 0, 0);
    layout->addWidget(label_connection_, 0, 1);
    layout->addWidget(new QLabel("Mode:", group), 1, 0);
    layout->addWidget(label_mode_, 1, 1);
    layout->addWidget(new QLabel("Accepts:", group), 2, 0);
    layout->addWidget(label_capabilities_, 2, 1);
    layout->addWidget(new QLabel("Elapsed time:", group), 3, 0);
    layout->addWidget(label_elapsed_time_, 3, 1);
    layout->setColumnStretch(1, 1);
    return group;
}

QGroupBox *MainWindow::_build_mode_group()
{
    auto group = new QGroupBox("High-level mode", this);
    auto layout = new QVBoxLayout(group);
    radio_idle_ = new QRadioButton("IDLE", group);
    radio_standing_ = new QRadioButton("STANDING", group);
    radio_walking_ = new QRadioButton("WALKING", group);
    radio_idle_->setObjectName("radio_idle");
    radio_standing_->setObjectName("radio_standing");
    radio_walking_->setObjectName("radio_walking");
    button_group_mode_ = new QButtonGroup(group);
    for (auto* radio : {radio_idle_, radio_standing_, radio_walking_})
    {
        // Enabled once get/info tells which modes the robot supports.
        radio->setEnabled(false);
        button_group_mode_->addButton(radio);
        layout->addWidget(radio);
    }
    auto note = new QLabel("The checked mode is the one reported by the robot.", group);
    note->setWordWrap(true);
    layout->addWidget(note);
    // buttonClicked is only emitted by the user, not by _update_mode() checking the reported mode.
    connect(button_group_mode_, &QButtonGroup::buttonClicked, this, [this](QAbstractButton* button) { _on_mode_clicked(button); });
    return group;
}

QSlider *MainWindow::_add_twist_row(QGridLayout *layout, const int &row, const QString &name, const double &max_value,
                                    QLabel *&value_label)
{
    auto slider = new QSlider(Qt::Horizontal, group_twist_);
    slider->setObjectName("slider_" + name.section(' ', 0, 0));  // e.g. slider_vx
    const int limit = static_cast<int>(max_value*slider_factor_);
    slider->setRange(-limit, limit);
    slider->setValue(0);
    value_label = new QLabel(_format(0.0), group_twist_);
    value_label->setMinimumWidth(60);
    layout->addWidget(new QLabel(name, group_twist_), row, 0);
    layout->addWidget(slider, row, 1);
    layout->addWidget(value_label, row, 2);
    connect(slider, &QSlider::valueChanged, this, [this](int) { _on_twist_slider_changed(); });
    return slider;
}

QGroupBox *MainWindow::_build_twist_group()
{
    group_twist_ = new QGroupBox("Target twist (body frame)", this);
    auto layout = new QGridLayout(group_twist_);
    slider_vx_ = _add_twist_row(layout, 0, "vx (m/s)", configuration_.max_vx, label_vx_);
    slider_vy_ = _add_twist_row(layout, 1, "vy (m/s)", configuration_.max_vy, label_vy_);
    slider_wz_ = _add_twist_row(layout, 2, "wz (rad/s)", configuration_.max_wz, label_wz_);
    auto button_zero = new QPushButton("ZERO", group_twist_);
    button_zero->setObjectName("button_zero_twist");
    button_zero->setStyleSheet("QPushButton { background-color: #fcca03; }");
    connect(button_zero, &QPushButton::clicked, this, [this]() { _zero_twist(); });
    layout->addWidget(button_zero, 3, 0, 1, 3);
    auto note = new QLabel("Sent every " + QString::number(time_step_in_milliseconds_) +
                           " ms while the robot accepts the twist (WALKING). The sliders go back to zero when it does not.",
                           group_twist_);
    note->setWordWrap(true);
    layout->addWidget(note, 4, 0, 1, 3);
    layout->setColumnStretch(1, 1);
    // Enabled while the robot accepts the twist.
    group_twist_->setEnabled(false);
    return group_twist_;
}

QGroupBox *MainWindow::_build_base_pose_group()
{
    group_base_pose_ = new QGroupBox("Target base pose (STANDING)", this);
    auto layout = new QGridLayout(group_base_pose_);
    const auto add_row = [this, layout](const int& row, const QString& name, QLabel*& value_label) {
        auto slider = new QSlider(Qt::Horizontal, group_base_pose_);
        slider->setObjectName("slider_" + name.section(' ', 0, 0));  // e.g. slider_roll
        slider->setRange(0, 0);  // The orientation limits arrive with get/info.
        value_label = new QLabel(_format(0.0), group_base_pose_);
        value_label->setMinimumWidth(60);
        layout->addWidget(new QLabel(name, group_base_pose_), row, 0);
        layout->addWidget(slider, row, 1);
        layout->addWidget(value_label, row, 2);
        return slider;
    };
    slider_roll_ = add_row(0, "roll (rad)", label_roll_);
    slider_pitch_ = add_row(1, "pitch (rad)", label_pitch_);
    slider_yaw_ = add_row(2, "yaw (rad)", label_yaw_);
    slider_height_ = add_row(3, "height (m)", label_height_);
    for (auto* slider : {slider_roll_, slider_pitch_, slider_yaw_})
        connect(slider, &QSlider::valueChanged, this, [this](int) { _on_base_orientation_slider_changed(); });
    connect(slider_height_, &QSlider::valueChanged, this, [this](int) { _on_base_height_slider_changed(); });

    auto button_neutral = new QPushButton("NEUTRAL", group_base_pose_);
    button_neutral->setObjectName("button_neutral_base_pose");
    button_neutral->setStyleSheet("QPushButton { background-color: #fcca03; }");
    connect(button_neutral, &QPushButton::clicked, this, [this]() { _neutral_base_pose(); });
    layout->addWidget(button_neutral, 4, 0, 1, 3);
    auto note = new QLabel("Angles relative to the pose at the start of STANDING (limits from get/info). The sliders "
                           "go back to the neutral pose when the robot does not accept them.", group_base_pose_);
    note->setWordWrap(true);
    layout->addWidget(note, 5, 0, 1, 3);
    layout->setColumnStretch(1, 1);
    group_base_pose_->setEnabled(false);
    _neutral_base_pose();
    return group_base_pose_;
}

QGroupBox *MainWindow::_build_imu_group()
{
    auto group = new QGroupBox("IMU (get/imu)", this);
    auto layout = new QGridLayout(group);
    label_quaternion_ = new QLabel("-", group);
    label_rpy_ = new QLabel("-", group);
    label_gyroscope_ = new QLabel("-", group);
    label_accelerometer_ = new QLabel("-", group);
    layout->addWidget(new QLabel("Orientation (w, x, y, z):", group), 0, 0);
    layout->addWidget(label_quaternion_, 0, 1);
    layout->addWidget(new QLabel("Roll, pitch, yaw (deg):", group), 1, 0);
    layout->addWidget(label_rpy_, 1, 1);
    layout->addWidget(new QLabel("Angular velocity (rad/s):", group), 2, 0);
    layout->addWidget(label_gyroscope_, 2, 1);
    layout->addWidget(new QLabel("Linear acceleration (m/s²):", group), 3, 0);
    layout->addWidget(label_accelerometer_, 3, 1);
    layout->setColumnStretch(1, 1);
    return group;
}

void MainWindow::_build_limb_group(LimbWidgets &limb)
{
    const QString title = limb.own_driver ? QString(limb.name.c_str()) + " (own driver)" : QString(limb.name.c_str());
    limb.group = new QGroupBox(title, layout_limbs_->parentWidget());
    auto layout = new QGridLayout(limb.group);

    limb.status = new QLabel(limb.group);
    layout->addWidget(limb.status, 0, 0, 1, 6);
    int row = 1;
    if (limb.own_driver)
    {
        // The legged driver does not gate this limb, so the user decides when it may move.
        limb.allow_commands = new QCheckBox("Allow commands (the mode of the base does not gate this limb)", limb.group);
        limb.allow_commands->setObjectName(QString("allow_commands_") + limb.name.c_str());
        limb.allow_commands->setEnabled(!monitor_only_);
        layout->addWidget(limb.allow_commands, row++, 0, 1, 6);
    }

    const QStringList headers = {"Joint", "Position (rad)", "Velocity (rad/s)", "Torque (Nm)", "Target", ""};
    for (int column = 0; column < headers.size(); column++)
        layout->addWidget(new QLabel("<b>" + headers.at(column) + "</b>", limb.group), row, column);
    row++;

    const std::size_t limb_index = static_cast<std::size_t>(&limb - &limbs_.front());
    for (const auto& joint_name : limb.joint_names)
    {
        JointWidgets joint;
        joint.position = new QLabel("-", limb.group);
        joint.velocity = new QLabel("-", limb.group);
        joint.torque = new QLabel("-", limb.group);
        joint.target = new QSlider(Qt::Horizontal, limb.group);
        joint.target->setObjectName(QString("target_") + limb.name.c_str() + "_" + QString::number(limb.joints.size()));
        joint.target->setRange(-3142, 3142);  // Until the joint limits arrive.
        joint.target->setEnabled(false);
        joint.target->setMinimumWidth(100);
        joint.target_value = new QLabel("-", limb.group);
        joint.target_value->setMinimumWidth(50);

        layout->addWidget(new QLabel(joint_name.c_str(), limb.group), row, 0);
        layout->addWidget(joint.position, row, 1);
        layout->addWidget(joint.velocity, row, 2);
        layout->addWidget(joint.torque, row, 3);
        layout->addWidget(joint.target, row, 4);
        layout->addWidget(joint.target_value, row, 5);

        QLabel* target_value = joint.target_value;
        connect(joint.target, &QSlider::valueChanged, this, [this, target_value](int value) {
            target_value->setText(_format(static_cast<double>(value)/slider_factor_));
        });
        // sliderMoved is only emitted by the user, not when the sliders follow the measured positions.
        connect(joint.target, &QSlider::sliderMoved, this, [this, limb_index](int) { _on_limb_slider_moved(limb_index); });

        limb.joints.push_back(joint);
        row++;
    }
    layout->setColumnStretch(4, 1);

    const int index = layout_limbs_->count();
    layout_limbs_->addWidget(limb.group, index/2, index%2);
}

//---------------------------------------------------------------------------------//
//---------------------------------- Update ---------------------------------------//
//---------------------------------------------------------------------------------//

void MainWindow::timerEvent([[maybe_unused]] QTimerEvent *event)
{
    // Closing the window stops the twist (see the destructor).
    if (*break_loops_)
    {
        killTimer(timer_id_);
        timer_id_ = 0;
        close();
        return;
    }

    _update();
}

void MainWindow::_update()
{
    // An exception must not leave a Qt event handler, and a failure in one step must not stop the others:
    // above all, the twist must keep being sent, or the driver stops the robot after its twist timeout. The
    // clients throw, for instance, when a topic that must be published by a single driver has more than one
    // publisher.
    const auto step = [this](const std::function<void()>& function) {
        try
        {
            function();
        }
        catch (const std::exception& e)
        {
            _show_error(e.what());
        }
    };
    step([this]() { rclcpp::spin_some(node_); });
    step([this]() { _update_status(); });
    step([this]() { _update_mode(); });
    step([this]() { _send_target_twist(); });
    step([this]() { _update_imu(); });
    step([this]() { _create_legged_limbs(); });
    step([this]() { _update_limbs(); });
    step([this]() { _update_elapsed_time(); });
}

void MainWindow::_update_status()
{
    if (!legged_client_->is_enabled())
    {
        label_connection_->setText("<font color='#c62828'>Waiting for " + QString(configuration_.legged_robot_prefix.c_str()) +
                                   "/get/info and get/status</font>");
        label_mode_->setText("-");
        label_capabilities_->setText("-");
        return;
    }
    label_connection_->setText("<font color='#2e7d32'>Connected to " + QString(configuration_.legged_robot_prefix.c_str()) + "</font>");

    QStringList accepts;
    if (legged_client_->accepts_twist())
        accepts << "twist";
    if (legged_client_->accepts_base_height())
        accepts << "base height";
    if (legged_client_->accepts_base_orientation())
        accepts << "base orientation";
    for (const auto& name : legged_client_->get_limb_names())
        if (legged_client_->is_limb_commandable(name))
            accepts << QString(name.c_str());
    label_capabilities_->setText(accepts.isEmpty() ? "nothing" : accepts.join(", "));
}

void MainWindow::_update_mode()
{
    if (!legged_client_->is_enabled())
        return;

    QRadioButton* radio{nullptr};
    QString name;
    switch (legged_client_->get_high_level_mode())
    {
    case HIGH_LEVEL_MODE::IDLE:
        radio = radio_idle_;
        name = "IDLE";
        break;
    case HIGH_LEVEL_MODE::STANDING:
        radio = radio_standing_;
        name = "STANDING";
        break;
    case HIGH_LEVEL_MODE::WALKING:
        radio = radio_walking_;
        name = "WALKING";
        break;
    }
    label_mode_->setText(name);
    if (radio && !radio->isChecked())
        radio->setChecked(true);

    // Only the modes the robot supports can be requested.
    const auto supported_modes = legged_client_->get_supported_high_level_modes();
    const auto is_supported = [&supported_modes](const HIGH_LEVEL_MODE& mode) {
        return std::find(supported_modes.begin(), supported_modes.end(), mode) != supported_modes.end();
    };
    radio_idle_->setEnabled(!monitor_only_ && is_supported(HIGH_LEVEL_MODE::IDLE));
    radio_standing_->setEnabled(!monitor_only_ && is_supported(HIGH_LEVEL_MODE::STANDING));
    radio_walking_->setEnabled(!monitor_only_ && is_supported(HIGH_LEVEL_MODE::WALKING));

    // The sliders go back to zero whenever the twist is not accepted, so that the robot never starts walking
    // with an old twist when WALKING is entered again.
    const bool accepts_twist = legged_client_->accepts_twist();
    if (accepted_twist_ && !accepts_twist)
        _zero_twist();
    accepted_twist_ = accepts_twist;
    group_twist_->setEnabled(!monitor_only_ && accepts_twist);

    // The base pose: the orientation limits arrive with get/info.
    if (!base_orientation_limits_configured_)
    {
        const auto [min_rpy, max_rpy] = legged_client_->get_base_orientation_limits();
        const std::vector<QSlider*> sliders{slider_roll_, slider_pitch_, slider_yaw_};
        for (std::size_t i = 0; i < sliders.size(); i++)
        {
            const long j = static_cast<long>(i);
            // One step (0.001 rad) inside the limits: at the limit itself, the conversion to a quaternion and back
            // can exceed it slightly, and the driver would warn that it clamped the target.
            const int lower = static_cast<int>(min_rpy(j)*slider_factor_);
            const int upper = static_cast<int>(max_rpy(j)*slider_factor_);
            sliders.at(i)->setRange(lower < 0 ? lower + 1 : lower, upper > 0 ? upper - 1 : upper);
            sliders.at(i)->setToolTip("[" + _format(min_rpy(j)) + ", " + _format(max_rpy(j)) + "] rad");
        }
        base_orientation_limits_configured_ = true;
        _neutral_base_pose();
    }
    if (!base_height_limits_configured_)
    {
        const auto [min_height, max_height, nominal_height] = legged_client_->get_base_height_limits();
        // Rounded inward, so that the integer range never exceeds the limits.
        slider_height_->setRange(static_cast<int>(std::ceil(min_height*slider_factor_)),
                                 static_cast<int>(std::floor(max_height*slider_factor_)));
        slider_height_->setToolTip("[" + _format(min_height) + ", " + _format(max_height) + "] m, nominal " +
                                   _format(nominal_height) + " m");
        nominal_base_height_ = nominal_height;
        base_height_limits_configured_ = true;
        _neutral_base_pose();
    }
    const bool accepts_base_orientation = legged_client_->accepts_base_orientation();
    const bool accepts_base_height = legged_client_->accepts_base_height();
    const bool accepts_base_pose = accepts_base_orientation || accepts_base_height;
    if (accepted_base_pose_ && !accepts_base_pose)
        _neutral_base_pose();
    accepted_base_pose_ = accepts_base_pose;
    group_base_pose_->setEnabled(!monitor_only_ && accepts_base_pose);
    for (auto* slider : {slider_roll_, slider_pitch_, slider_yaw_})
        slider->setEnabled(accepts_base_orientation);
    slider_height_->setEnabled(accepts_base_height);
}

void MainWindow::_update_imu()
{
    if (!legged_client_->is_enabled())
        return;

    DQ orientation, angular_velocity, linear_acceleration;
    try
    {
        orientation = legged_client_->get_orientation();
        angular_velocity = legged_client_->get_angular_velocity();
        linear_acceleration = legged_client_->get_linear_acceleration();
    }
    catch (const std::runtime_error&)
    {
        return;  // Nothing received on get/imu yet.
    }

    const VectorXd r = orientation.vec4();
    label_quaternion_->setText(_format(r(0), 4) + ", " + _format(r(1), 4) + ", " + _format(r(2), 4) + ", " + _format(r(3), 4));
    if (is_unit(orientation))
    {
        const Eigen::Vector3d rpy = sas::unit_quaternion_to_rpy(orientation)*180.0/M_PI;
        label_rpy_->setText(_format(rpy(0), 2) + ", " + _format(rpy(1), 2) + ", " + _format(rpy(2), 2));
    }
    const VectorXd w = angular_velocity.vec3();
    label_gyroscope_->setText(_format(w(0)) + ", " + _format(w(1)) + ", " + _format(w(2)));
    const VectorXd a = linear_acceleration.vec3();
    label_accelerometer_->setText(_format(a(0)) + ", " + _format(a(1)) + ", " + _format(a(2)));
}

void MainWindow::_create_legged_limbs()
{
    if (legged_limbs_created_ || !legged_client_->is_enabled())
        return;

    // The limbs of the legged driver are listed before the limbs with their own driver, as in the
    // limb_prefixes of a controller.
    std::vector<LimbWidgets> legged_limbs;
    for (const auto& name : legged_client_->get_limb_names())
    {
        LimbWidgets limb;
        limb.name = name;
        limb.joint_names = legged_client_->get_limb_joint_names(name);
        limb.client = std::make_shared<sas::RobotDriverClient>(node_, configuration_.legged_robot_prefix + "/" + name, blacklist_);
        legged_limbs.push_back(std::move(limb));
    }
    // The widgets of the limbs with their own driver are only built after this (see _update_limbs()), so the
    // indices captured by the slider callbacks do not change.
    limbs_.insert(limbs_.begin(), std::make_move_iterator(legged_limbs.begin()), std::make_move_iterator(legged_limbs.end()));
    for (auto& limb : limbs_)
        if (!limb.own_driver)
            _build_limb_group(limb);

    legged_limbs_created_ = true;
    statusBar()->showMessage("Connected to " + QString(configuration_.legged_robot_prefix.c_str()) + ".", 5000);
}

void MainWindow::_update_limbs()
{
    // The limbs with their own driver wait for the limbs of the legged driver, so that the widgets keep the
    // order of the limbs.
    if (!legged_limbs_created_)
        return;

    for (std::size_t limb_index = 0; limb_index < limbs_.size(); limb_index++)
    {
        auto& limb = limbs_.at(limb_index);
        if (!limb.client->is_enabled())
        {
            if (limb.status)
                limb.status->setText("<font color='#c62828'>Waiting for " + QString(limb.client->get_topic_prefix().c_str()) +
                                     "/get/joint_states</font>");
            continue;
        }

        const VectorXd q = limb.client->get_joint_positions();
        if (!limb.group)
        {
            // A limb with its own driver: its number of joints is known from its joint states.
            for (int i = 0; i < q.size(); i++)
                limb.joint_names.push_back("joint " + std::to_string(i));
            _build_limb_group(limb);
        }

        if (static_cast<std::size_t>(q.size()) != limb.joints.size())
        {
            limb.status->setText("<font color='#c62828'>" + QString::number(q.size()) + " joint positions, but " +
                                 QString::number(limb.joints.size()) + " joints in get/info</font>");
            continue;
        }

        const auto [q_min, q_max] = limb.client->get_joint_limits();
        if (!limb.limits_configured && q_min.size() == q.size() && q_max.size() == q.size())
        {
            for (std::size_t i = 0; i < limb.joints.size(); i++)
            {
                const long j = static_cast<long>(i);
                // Truncated toward zero, so that the integer range never exceeds the joint limits.
                limb.joints.at(i).target->setRange(static_cast<int>(q_min(j)*slider_factor_),
                                                   static_cast<int>(q_max(j)*slider_factor_));
                limb.joints.at(i).target->setToolTip("[" + _format(q_min(j)) + ", " + _format(q_max(j)) + "] rad");
            }
            limb.limits_configured = true;
        }

        // A driver does not have to publish the joint velocities and torques, and the client throws without them.
        VectorXd q_dot;
        VectorXd tau;
        try{q_dot = limb.client->get_joint_velocities();} catch(const std::exception&){}
        try{tau = limb.client->get_joint_forces();} catch(const std::exception&){}
        const bool commandable = _is_commandable(limb);
        for (std::size_t i = 0; i < limb.joints.size(); i++)
        {
            const long j = static_cast<long>(i);
            auto& joint = limb.joints.at(i);
            joint.position->setText(_format(q(j)));
            joint.velocity->setText(q_dot.size() == q.size() ? _format(q_dot(j)) : "-");
            joint.torque->setText(tau.size() == q.size() ? _format(tau(j)) : "-");
            joint.target->setEnabled(commandable);
            // While not commandable, the sliders follow the robot, so that the first drag starts from the
            // current pose instead of moving the other joints of the limb.
            if (!commandable)
                joint.target->setValue(static_cast<int>(std::lround(q(j)*slider_factor_)));
            joint.target_value->setText(_format(joint.target->value()/slider_factor_));
        }
        limb.status->setText(commandable ? "<font color='#2e7d32'>Accepts targets</font>"
                                         : "Does not accept targets");
    }
}

void MainWindow::_update_elapsed_time()
{
    elapsed_time_ += time_step_in_milliseconds_;
    const int total_seconds = static_cast<int>(elapsed_time_/1000.0);
    label_elapsed_time_->setText(QString("%1:%2:%3")
                                     .arg(total_seconds/3600, 2, 10, QChar('0'))
                                     .arg((total_seconds%3600)/60, 2, 10, QChar('0'))
                                     .arg(total_seconds%60, 2, 10, QChar('0')));
}

//---------------------------------------------------------------------------------//
//---------------------------------- Limbs ----------------------------------------//
//---------------------------------------------------------------------------------//

bool MainWindow::_is_commandable(const LimbWidgets &limb) const
{
    if (monitor_only_ || !limb.group || !limb.client->is_enabled())
        return false;
    if (limb.own_driver)
        return limb.allow_commands && limb.allow_commands->isChecked();
    // The legged driver decides it (get/status). A target sent to a limb that does not accept it would be kept
    // by the driver of the limb, so none is sent.
    return legged_client_->is_enabled() && legged_client_->is_limb_commandable(limb.name);
}

void MainWindow::_on_limb_slider_moved(const std::size_t &limb_index)
{
    try
    {
        const auto& limb = limbs_.at(limb_index);
        if (!_is_commandable(limb))
            return;

        // sliderPosition() instead of value(): sliderMoved is emitted before value() is updated.
        VectorXd target(static_cast<long>(limb.joints.size()));
        for (std::size_t i = 0; i < limb.joints.size(); i++)
            target(static_cast<long>(i)) = static_cast<double>(limb.joints.at(i).target->sliderPosition())/slider_factor_;
        limb.client->send_target_joint_positions(target);
    }
    catch (const std::exception& e)
    {
        _show_error(e.what());
    }
}

//---------------------------------------------------------------------------------//
//--------------------------------- Commands --------------------------------------//
//---------------------------------------------------------------------------------//

void MainWindow::_on_mode_clicked(QAbstractButton *button)
{
    try
    {
        _zero_twist();
        _neutral_base_pose();
        if (monitor_only_ || !legged_client_->is_enabled())
            return;

        if (button == radio_idle_)
            legged_client_->send_high_level_mode(HIGH_LEVEL_MODE::IDLE);
        else if (button == radio_standing_)
            legged_client_->send_high_level_mode(HIGH_LEVEL_MODE::STANDING);
        else if (button == radio_walking_)
            legged_client_->send_high_level_mode(HIGH_LEVEL_MODE::WALKING);
        statusBar()->showMessage("Mode " + button->text() + " requested.", 3000);
    }
    catch (const std::exception& e)
    {
        _show_error(e.what());
    }
}

void MainWindow::_on_twist_slider_changed()
{
    label_vx_->setText(_format(slider_vx_->value()/slider_factor_));
    label_vy_->setText(_format(slider_vy_->value()/slider_factor_));
    label_wz_->setText(_format(slider_wz_->value()/slider_factor_));
    try
    {
        _send_target_twist();
    }
    catch (const std::exception& e)
    {
        _show_error(e.what());
    }
}

void MainWindow::_send_target_twist()
{
    if (monitor_only_ || !legged_client_->is_enabled() || !legged_client_->accepts_twist())
        return;

    const double vx = slider_vx_->value()/slider_factor_;
    const double vy = slider_vy_->value()/slider_factor_;
    const double wz = slider_wz_->value()/slider_factor_;
    // twist = angular + E_*linear, in the body frame.
    legged_client_->send_target_twist(wz*k_ + E_*(vx*i_ + vy*j_));
}

void MainWindow::_zero_twist()
{
    // Each setValue() sends the twist (see _on_twist_slider_changed()); the last one sends a zero twist.
    slider_vx_->setValue(0);
    slider_vy_->setValue(0);
    slider_wz_->setValue(0);
}

void MainWindow::_on_base_orientation_slider_changed()
{
    const double roll = slider_roll_->value()/slider_factor_;
    const double pitch = slider_pitch_->value()/slider_factor_;
    const double yaw = slider_yaw_->value()/slider_factor_;
    label_roll_->setText(_format(roll));
    label_pitch_->setText(_format(pitch));
    label_yaw_->setText(_format(yaw));
    try
    {
        // The driver keeps the last target, so it is only sent when it changes.
        if (!monitor_only_ && legged_client_->is_enabled() && legged_client_->accepts_base_orientation())
            legged_client_->send_target_base_orientation_rpy(roll, pitch, yaw);
    }
    catch (const std::exception& e)
    {
        _show_error(e.what());
    }
}

void MainWindow::_on_base_height_slider_changed()
{
    const double height = slider_height_->value()/slider_factor_;
    label_height_->setText(_format(height));
    try
    {
        if (!monitor_only_ && legged_client_->is_enabled() && legged_client_->accepts_base_height())
            legged_client_->send_target_base_height(height);
    }
    catch (const std::exception& e)
    {
        _show_error(e.what());
    }
}

void MainWindow::_neutral_base_pose()
{
    // Each setValue() sends the target if the robot accepts it (see the slider callbacks).
    slider_roll_->setValue(0);
    slider_pitch_->setValue(0);
    slider_yaw_->setValue(0);
    slider_height_->setValue(static_cast<int>(std::lround(nominal_base_height_*slider_factor_)));
    _on_base_orientation_slider_changed();
    _on_base_height_slider_changed();
}

void MainWindow::_on_start_watchdog_clicked()
{
    if (watchdog_running_)
        return;
    QString text = QString("Start the watchdog of %1").arg(configuration_.legged_robot_prefix.c_str());
    for (const auto& prefix : configuration_.own_driver_limb_prefixes)
        text += QString(" and %1").arg(prefix.c_str());
    text += QString(" (period %1 s, maximum acceptable delay %2 s)?\n\n"
                    "It cannot be stopped: when this window closes, the triggers stop and the drivers stop the robot.")
                .arg(watchdog_period_in_seconds_).arg(configuration_.watchdog_maximum_acceptable_delay_in_seconds);
    if (!monitor_only_)
        text += "\n\nThis window also commands the robot (control mode), so the watchdog only detects that the "
                "window stopped. To watch a controller, run a second window in monitor mode.";
    if (QMessageBox::question(this, "Start the watchdog", text) != QMessageBox::Yes)
        return;

    try
    {
        // Only WATCHDOG_CONTROL: these clients neither command nor read the robot.
        const std::vector<MODE_BLACKLIST_FLAG> watchdog_only{MODE_BLACKLIST_FLAG::JOINT_CONTROL,
                                                             MODE_BLACKLIST_FLAG::JOINT_MONITORING};
        watchdog_clients_.push_back(std::make_shared<sas::RobotDriverClient>(node_, configuration_.legged_robot_prefix,
                                                                             watchdog_only));
        for (const auto& prefix : configuration_.own_driver_limb_prefixes)
            watchdog_clients_.push_back(std::make_shared<sas::RobotDriverClient>(node_, prefix, watchdog_only));
    }
    catch (const std::exception& e)
    {
        watchdog_clients_.clear();
        _show_error(e.what());
        return;
    }

    // The period is fixed from now on (the driver fixes it at the first trigger).
    slider_watchdog_period_->setEnabled(false);
    watchdog_running_ = true;
    watchdog_thread_ = std::thread([this]() { _watchdog_loop(); });
    button_watchdog_->setEnabled(false);
    label_watchdog_->setText(QString("<font color='#2e7d32'><b>Watchdog: running</b></font> (%1 s)")
                                 .arg(watchdog_period_in_seconds_));
    statusBar()->showMessage("Watchdog started.", 5000);
}

void MainWindow::_watchdog_loop()
{
    // Ten triggers per period, so that a late one does not trip the watchdog.
    // watchdog_period_in_seconds_ does not change once the thread runs (the slider is disabled).
    const double period = watchdog_period_in_seconds_;
    const auto interval = std::chrono::duration<double>(period/10.0);
    while (watchdog_running_)
    {
        for (const auto& client : watchdog_clients_)
        {
            try
            {
                client->send_watchdog_trigger(true, period,
                                              configuration_.watchdog_maximum_acceptable_delay_in_seconds);
            }
            catch (const std::exception& e)
            {
                std::cerr<<"MainWindow::_watchdog_loop. "<<e.what()<<std::endl;
            }
        }
        std::this_thread::sleep_for(interval);
    }
}

void MainWindow::_on_emergency_stop_clicked()
{
    // send_shutdown_signal() is available whatever the blacklist. The limbs with their own driver are separate
    // drivers, so the shutdown of the legged driver does not reach them. The drivers whose set/shutdown is remapped
    // (e.g. to the emergency stop of the lab) only receive it on shutdown_topic.
    try
    {
        legged_client_->send_shutdown_signal();
        for (const auto& limb : limbs_)
            if (limb.own_driver)
                limb.client->send_shutdown_signal();
        if (publisher_shutdown_)
        {
            sas_msgs::msg::Bool msg;
            msg.data = true;
            publisher_shutdown_->publish(msg);
        }
        statusBar()->showMessage("Shutdown signal sent.");
    }
    catch (const std::exception& e)
    {
        _show_error(e.what());
    }
}

void MainWindow::_show_error(const std::string &what)
{
    statusBar()->showMessage(QString("Error: ") + what.c_str(), 3000);
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "::" << what);
}

QString MainWindow::_format(const double &value, const int &decimals)
{
    return QString::number(value, 'f', decimals);
}
