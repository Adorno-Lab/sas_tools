#pragma once

#include <Eigen/Dense>
#include <dqrobotics/DQ.h>

namespace sas
{

/**
 * @brief rpy_to_unit_quaternion Returns the unit quaternion r = r_z(yaw)*r_y(pitch)*r_x(roll)
 *        (ZYX intrinsic rotation sequence).
 * @param roll  Rotation around the x-axis, in radians.
 * @param pitch Rotation around the y-axis, in radians.
 * @param yaw   Rotation around the z-axis, in radians.
 */
DQ_robotics::DQ rpy_to_unit_quaternion(const double& roll, const double& pitch, const double& yaw);

/**
 * @brief unit_quaternion_to_rpy Returns the ZYX roll-pitch-yaw angles of @p r, the inverse
 *        of rpy_to_unit_quaternion() for pitch in (-pi/2, pi/2).
 * @param r A quaternion (no dual part). It is normalized before the conversion.
 * @return {roll, pitch, yaw}, in radians.
 * @throws std::invalid_argument if @p r is not a non-zero quaternion.
 */
Eigen::Vector3d unit_quaternion_to_rpy(const DQ_robotics::DQ& r);

}
