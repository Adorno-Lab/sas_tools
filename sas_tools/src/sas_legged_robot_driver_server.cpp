#include <sas_tools/sas_legged_robot_driver_server.hpp>
#include <limits>
#include <stdexcept>

using std::placeholders::_1;

namespace sas
{

LeggedRobotDriverServer::LeggedRobotDriverServer(const std::shared_ptr<rclcpp::Node> &node,
                                                 const std::string &topic_prefix):
    node_(node),
    topic_prefix_(topic_prefix)
{
    subscriber_target_twist_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic_prefix_ + "/set/target_twist", 1,
        std::bind(&LeggedRobotDriverServer::_callback_target_twist, this, _1));
    subscriber_high_level_mode_ = node_->create_subscription<sas_legged_msgs::msg::HighLevelMode>(
        topic_prefix_ + "/set/high_level_mode", 1,
        std::bind(&LeggedRobotDriverServer::_callback_high_level_mode, this, _1));
    subscriber_target_base_height_ = node_->create_subscription<std_msgs::msg::Float64>(
        topic_prefix_ + "/set/target_base_height", 1,
        std::bind(&LeggedRobotDriverServer::_callback_target_base_height, this, _1));
    subscriber_target_base_orientation_ = node_->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        topic_prefix_ + "/set/target_base_orientation", 1,
        std::bind(&LeggedRobotDriverServer::_callback_target_base_orientation, this, _1));

    publisher_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_prefix_ + "/get/imu", 1);
    publisher_status_ = node_->create_publisher<sas_legged_msgs::msg::LeggedRobotStatus>(topic_prefix_ + "/get/status", 1);
    publisher_info_ = node_->create_publisher<sas_legged_msgs::msg::LeggedRobotInfo>(
        topic_prefix_ + "/get/info", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
}

void LeggedRobotDriverServer::_check_exclusive_publisher(const std::string &topic) const
{
    if(node_->count_publishers(topic)>1)
        throw std::runtime_error(topic + " must be exclusively published and there is more than one publisher connected.");
}

void LeggedRobotDriverServer::_callback_target_twist(const geometry_msgs::msg::TwistStamped &msg)
{
    _check_exclusive_publisher(topic_prefix_ + "/set/target_twist");
    target_twist_ = DQ(0, msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,
                       0, msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z);
    target_twist_received_ = true;
    target_twist_time_point_ = std::chrono::steady_clock::now();
}

void LeggedRobotDriverServer::_callback_high_level_mode(const sas_legged_msgs::msg::HighLevelMode &msg)
{
    _check_exclusive_publisher(topic_prefix_ + "/set/high_level_mode");
    if (msg.mode > sas_legged_msgs::msg::HighLevelMode::WALKING)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), topic_prefix_ << "/set/high_level_mode: ignoring the invalid mode "
                                                 << static_cast<int>(msg.mode) << ".");
        return;
    }
    target_high_level_mode_ = static_cast<LeggedRobotDriver::HIGH_LEVEL_MODE>(msg.mode);
    new_target_high_level_mode_ = true;
}

void LeggedRobotDriverServer::_callback_target_base_height(const std_msgs::msg::Float64 &msg)
{
    _check_exclusive_publisher(topic_prefix_ + "/set/target_base_height");
    target_base_height_ = msg.data;
    new_target_base_height_ = true;
}

void LeggedRobotDriverServer::_callback_target_base_orientation(const geometry_msgs::msg::QuaternionStamped &msg)
{
    _check_exclusive_publisher(topic_prefix_ + "/set/target_base_orientation");
    const DQ r(msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z);
    if (r.vec4().norm() == 0.0)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), topic_prefix_ << "/set/target_base_orientation: ignoring a zero quaternion.");
        return;
    }
    target_base_orientation_ = r.normalize();
    new_target_base_orientation_ = true;
}

bool LeggedRobotDriverServer::has_received_target_twist() const
{
    return target_twist_received_;
}

DQ LeggedRobotDriverServer::get_target_twist() const
{
    return target_twist_;
}

double LeggedRobotDriverServer::get_seconds_since_last_target_twist() const
{
    if (!target_twist_received_)
        return std::numeric_limits<double>::infinity();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - target_twist_time_point_).count();
}

bool LeggedRobotDriverServer::has_new_target_high_level_mode() const
{
    return new_target_high_level_mode_;
}

LeggedRobotDriver::HIGH_LEVEL_MODE LeggedRobotDriverServer::get_target_high_level_mode()
{
    new_target_high_level_mode_ = false;
    return target_high_level_mode_;
}

bool LeggedRobotDriverServer::has_new_target_base_height() const
{
    return new_target_base_height_;
}

double LeggedRobotDriverServer::get_target_base_height()
{
    new_target_base_height_ = false;
    return target_base_height_;
}

bool LeggedRobotDriverServer::has_new_target_base_orientation() const
{
    return new_target_base_orientation_;
}

DQ LeggedRobotDriverServer::get_target_base_orientation()
{
    new_target_base_orientation_ = false;
    return target_base_orientation_;
}

void LeggedRobotDriverServer::send_imu(const DQ &orientation, const DQ &angular_velocity, const DQ &linear_acceleration)
{
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = node_->get_clock()->now();

    const VectorXd r = orientation.vec4();
    msg.orientation.w = r(0);
    msg.orientation.x = r(1);
    msg.orientation.y = r(2);
    msg.orientation.z = r(3);

    const VectorXd w = angular_velocity.vec3();
    msg.angular_velocity.x = w(0);
    msg.angular_velocity.y = w(1);
    msg.angular_velocity.z = w(2);

    const VectorXd a = linear_acceleration.vec3();
    msg.linear_acceleration.x = a(0);
    msg.linear_acceleration.y = a(1);
    msg.linear_acceleration.z = a(2);

    publisher_imu_->publish(msg);
}

void LeggedRobotDriverServer::send_status(const LeggedRobotDriver::HIGH_LEVEL_MODE &mode,
                                          const std::vector<bool> &commandable_joints,
                                          const LeggedRobotDriver::CommandAcceptance &acceptance)
{
    sas_legged_msgs::msg::LeggedRobotStatus msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.mode = static_cast<uint8_t>(mode);
    msg.commandable_joints = commandable_joints;
    msg.accepts_twist = acceptance.twist;
    msg.accepts_base_orientation = acceptance.base_orientation;
    msg.accepts_base_height = acceptance.base_height;
    msg.accepts_manipulator_commands = acceptance.manipulators;
    publisher_status_->publish(msg);
}

void LeggedRobotDriverServer::send_info(const sas_legged_msgs::msg::LeggedRobotInfo &info)
{
    publisher_info_->publish(info);
}

}
