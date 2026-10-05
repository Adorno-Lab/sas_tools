#include <sas_tools/rpy_conversions.hpp>
#include <cmath>
#include <stdexcept>

using namespace DQ_robotics;

namespace sas
{

DQ rpy_to_unit_quaternion(const double &roll, const double &pitch, const double &yaw)
{
    const DQ r_x = std::cos(roll/2.0)  + i_*std::sin(roll/2.0);
    const DQ r_y = std::cos(pitch/2.0) + j_*std::sin(pitch/2.0);
    const DQ r_z = std::cos(yaw/2.0)   + k_*std::sin(yaw/2.0);
    return r_z*r_y*r_x;
}

Eigen::Vector3d unit_quaternion_to_rpy(const DQ &r)
{
    if (!is_quaternion(r) || r.vec4().norm() == 0.0)
        throw std::invalid_argument("unit_quaternion_to_rpy: expected a non-zero quaternion.");

    const Eigen::Vector4d rv = r.vec4().normalized();
    const Eigen::Quaterniond q(rv(0), rv(1), rv(2), rv(3));
    const Eigen::Matrix3d R = q.toRotationMatrix();

    const double roll  = std::atan2(R(2, 1), R(2, 2));
    const double pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));
    const double yaw   = std::atan2(R(1, 0), R(0, 0));

    return {roll, pitch, yaw};
}

}
