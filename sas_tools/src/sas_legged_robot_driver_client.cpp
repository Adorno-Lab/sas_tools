#include <sas_tools/sas_legged_robot_driver_client.hpp>
#include <sas_tools/rpy_conversions.hpp>
#include <algorithm>
#include <stdexcept>

using std::placeholders::_1;

namespace sas
{

LeggedRobotDriverClient::LeggedRobotDriverClient(const std::shared_ptr<rclcpp::Node> &node,
                                                 const std::string &topic_prefix,
                                                 const std::vector<MODE_BLACKLIST_FLAG> &blacklisted_modes):
    RobotDriverClient(node, topic_prefix, blacklisted_modes),
    node_(node),
    topic_prefix_(topic_prefix),
    blacklisted_modes_(blacklisted_modes)
{
    if(!_is_blacklisted(MODE_BLACKLIST_FLAG::JOINT_CONTROL))
    {
        publisher_target_twist_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_prefix_ + "/set/target_twist", 1);
        publisher_high_level_mode_ = node_->create_publisher<sas_legged_msgs::msg::HighLevelMode>(topic_prefix_ + "/set/high_level_mode", 1);
        publisher_target_base_height_ = node_->create_publisher<std_msgs::msg::Float64>(topic_prefix_ + "/set/target_base_height", 1);
        publisher_target_base_orientation_ = node_->create_publisher<geometry_msgs::msg::QuaternionStamped>(topic_prefix_ + "/set/target_base_orientation", 1);
    }

    if(!_is_blacklisted(MODE_BLACKLIST_FLAG::JOINT_MONITORING))
    {
        subscriber_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            topic_prefix_ + "/get/imu", 1, std::bind(&LeggedRobotDriverClient::_callback_imu, this, _1));
        subscriber_status_ = node_->create_subscription<sas_legged_msgs::msg::LeggedRobotStatus>(
            topic_prefix_ + "/get/status", 1, std::bind(&LeggedRobotDriverClient::_callback_status, this, _1));
        subscriber_info_ = node_->create_subscription<sas_legged_msgs::msg::LeggedRobotInfo>(
            topic_prefix_ + "/get/info", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&LeggedRobotDriverClient::_callback_info, this, _1));
    }
}

bool LeggedRobotDriverClient::_is_blacklisted(const MODE_BLACKLIST_FLAG &mode) const
{
    return std::find(blacklisted_modes_.begin(), blacklisted_modes_.end(), mode) != blacklisted_modes_.end();
}

void LeggedRobotDriverClient::_check_not_blacklisted(const MODE_BLACKLIST_FLAG &mode, const std::string &function) const
{
    if(_is_blacklisted(mode))
        throw std::runtime_error("LeggedRobotDriverClient::" + function + "::This method is blacklisted");
}

void LeggedRobotDriverClient::_check_exclusive_publisher(const std::string &topic) const
{
    if(node_->count_publishers(topic)>1)
        throw std::runtime_error(topic + " must be exclusively published and there is more than one publisher connected.");
}

void LeggedRobotDriverClient::_check_received(const bool &received, const std::string &topic, const std::string &function) const
{
    _check_not_blacklisted(MODE_BLACKLIST_FLAG::JOINT_MONITORING, function);
    if(!received)
        throw std::runtime_error(topic_prefix_ + "::LeggedRobotDriverClient::" + function + "::nothing received on " + topic + " yet.");
}

void LeggedRobotDriverClient::_callback_imu(const sensor_msgs::msg::Imu &msg)
{
    _check_exclusive_publisher(topic_prefix_ + "/get/imu");
    orientation_ = DQ(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    angular_velocity_ = DQ(0, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    linear_acceleration_ = DQ(0, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    imu_received_ = true;
}

void LeggedRobotDriverClient::_callback_status(const sas_legged_msgs::msg::LeggedRobotStatus &msg)
{
    _check_exclusive_publisher(topic_prefix_ + "/get/status");
    status_ = msg;
    status_received_ = true;
}

void LeggedRobotDriverClient::_callback_info(const sas_legged_msgs::msg::LeggedRobotInfo &msg)
{
    _check_exclusive_publisher(topic_prefix_ + "/get/info");
    info_ = msg;
    info_received_ = true;
}

void LeggedRobotDriverClient::send_target_twist(const DQ &twist)
{
    _check_not_blacklisted(MODE_BLACKLIST_FLAG::JOINT_CONTROL, __FUNCTION__);
    const VectorXd v = twist.vec6();  // {wx, wy, wz, vx, vy, vz}
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.twist.angular.x = v(0);
    msg.twist.angular.y = v(1);
    msg.twist.angular.z = v(2);
    msg.twist.linear.x  = v(3);
    msg.twist.linear.y  = v(4);
    msg.twist.linear.z  = v(5);
    publisher_target_twist_->publish(msg);
}

void LeggedRobotDriverClient::send_high_level_mode(const LeggedRobotDriver::HIGH_LEVEL_MODE &mode)
{
    _check_not_blacklisted(MODE_BLACKLIST_FLAG::JOINT_CONTROL, __FUNCTION__);
    sas_legged_msgs::msg::HighLevelMode msg;
    msg.mode = static_cast<uint8_t>(mode);
    publisher_high_level_mode_->publish(msg);
}

void LeggedRobotDriverClient::send_target_base_height(const double &base_height)
{
    _check_not_blacklisted(MODE_BLACKLIST_FLAG::JOINT_CONTROL, __FUNCTION__);
    std_msgs::msg::Float64 msg;
    msg.data = base_height;
    publisher_target_base_height_->publish(msg);
}

void LeggedRobotDriverClient::send_target_base_orientation(const DQ &r)
{
    _check_not_blacklisted(MODE_BLACKLIST_FLAG::JOINT_CONTROL, __FUNCTION__);
    if(!is_quaternion(r) || !is_unit(r))
        throw std::invalid_argument("LeggedRobotDriverClient::send_target_base_orientation: expected a unit quaternion.");
    const VectorXd q = r.vec4();
    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.quaternion.w = q(0);
    msg.quaternion.x = q(1);
    msg.quaternion.y = q(2);
    msg.quaternion.z = q(3);
    publisher_target_base_orientation_->publish(msg);
}

void LeggedRobotDriverClient::send_target_base_orientation_rpy(const double &roll, const double &pitch, const double &yaw)
{
    send_target_base_orientation(rpy_to_unit_quaternion(roll, pitch, yaw));
}

DQ LeggedRobotDriverClient::get_orientation() const
{
    _check_received(imu_received_, "get/imu", __FUNCTION__);
    return orientation_;
}

DQ LeggedRobotDriverClient::get_angular_velocity() const
{
    _check_received(imu_received_, "get/imu", __FUNCTION__);
    return angular_velocity_;
}

DQ LeggedRobotDriverClient::get_linear_acceleration() const
{
    _check_received(imu_received_, "get/imu", __FUNCTION__);
    return linear_acceleration_;
}

LeggedRobotDriver::HIGH_LEVEL_MODE LeggedRobotDriverClient::get_high_level_mode() const
{
    _check_received(status_received_, "get/status", __FUNCTION__);
    return static_cast<LeggedRobotDriver::HIGH_LEVEL_MODE>(status_.mode);
}

std::vector<bool> LeggedRobotDriverClient::get_commandable_joint_mask() const
{
    _check_received(status_received_, "get/status", __FUNCTION__);
    return status_.commandable_joints;
}

bool LeggedRobotDriverClient::accepts_twist() const
{
    _check_received(status_received_, "get/status", __FUNCTION__);
    return status_.accepts_twist;
}

bool LeggedRobotDriverClient::accepts_base_orientation() const
{
    _check_received(status_received_, "get/status", __FUNCTION__);
    return status_.accepts_base_orientation;
}

bool LeggedRobotDriverClient::accepts_base_height() const
{
    _check_received(status_received_, "get/status", __FUNCTION__);
    return status_.accepts_base_height;
}

bool LeggedRobotDriverClient::accepts_manipulator_commands() const
{
    _check_received(status_received_, "get/status", __FUNCTION__);
    return status_.accepts_manipulator_commands;
}

std::string LeggedRobotDriverClient::get_robot_model() const
{
    _check_received(info_received_, "get/info", __FUNCTION__);
    return info_.robot_model;
}

std::vector<std::string> LeggedRobotDriverClient::get_joint_names() const
{
    _check_received(info_received_, "get/info", __FUNCTION__);
    return info_.joint_names;
}

std::vector<LeggedRobotDriver::HIGH_LEVEL_MODE> LeggedRobotDriverClient::get_supported_high_level_modes() const
{
    _check_received(info_received_, "get/info", __FUNCTION__);
    std::vector<LeggedRobotDriver::HIGH_LEVEL_MODE> modes;
    for (const auto& mode : info_.supported_modes)
        modes.push_back(static_cast<LeggedRobotDriver::HIGH_LEVEL_MODE>(mode));
    return modes;
}

bool LeggedRobotDriverClient::is_supported(const LeggedRobotDriver::LeggedFunctionality &functionality) const
{
    _check_received(info_received_, "get/info", __FUNCTION__);
    switch (functionality)
    {
    case LeggedRobotDriver::LeggedFunctionality::Twist:                    return info_.supports_twist;
    case LeggedRobotDriver::LeggedFunctionality::BaseHeight:               return info_.supports_base_height;
    case LeggedRobotDriver::LeggedFunctionality::BaseOrientation:          return info_.supports_base_orientation;
    case LeggedRobotDriver::LeggedFunctionality::ManipulationWhileWalking: return info_.supports_manipulation_while_walking;
    }
    return false;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> LeggedRobotDriverClient::get_base_orientation_limits() const
{
    _check_received(info_received_, "get/info", __FUNCTION__);
    return {Eigen::Vector3d(info_.min_base_roll, info_.min_base_pitch, info_.min_base_yaw),
            Eigen::Vector3d(info_.max_base_roll, info_.max_base_pitch, info_.max_base_yaw)};
}

bool LeggedRobotDriverClient::is_enabled(const RobotDriver::Functionality &supported_functionality) const
{
    return RobotDriverClient::is_enabled(supported_functionality) && info_received_ && status_received_;
}

}
