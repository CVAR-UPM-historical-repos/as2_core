/*!*******************************************************************************************
 *  \file       frame_utils.hpp
 *  \brief      Aerostack2 frame utils header file.
 *  \authors    Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef __AEROSTACK2_CODE_UTILS_HPP__
#define __AEROSTACK2_CODE_UTILS_HPP__

#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>

namespace as2 {
namespace frame {
/**
 * @brief Convert ENU (east, north, up) to FLU (forward, left, up) frame.
 *
 * @param quaternion tf2::Quaternion in ENU frame.
 * @param enu_vec Eigen::Vector3d ENU vector.
 * @return Eigen::Vector3d FLU vector.
 */
Eigen::Vector3d convertENUtoFLU(const float roll_angle,
                                const float pitch_angle,
                                const float yaw_angle,
                                const Eigen::Vector3d &enu_vec);

/**
 * @brief Convert ENU (east, north, up) to FLU (forward, left, up) frame.
 *
 * @param quaternion tf2::Quaternion in ENU frame.
 * @param enu_vec Eigen::Vector3d ENU vector.
 * @return Eigen::Vector3d FLU vector.
 */
Eigen::Vector3d convertENUtoFLU(const tf2::Quaternion &quaternion, const Eigen::Vector3d &enu_vec);

/**
 * @brief Convert ENU (east, north, up) to FLU (forward, left, up) frame.
 *
 * @param quaternion geometry_msgs::msg::Quaternion in ENU frame.
 * @param enu_vec Eigen::Vector3d ENU vector.
 * @return Eigen::Vector3d FLU vector.
 */
Eigen::Vector3d convertENUtoFLU(const geometry_msgs::msg::Quaternion &quaternion,
                                const Eigen::Vector3d &enu_vec);

/**
 * @brief Convert ENU (east, north, up) to FLU (forward, left, up) frame.
 *
 * @param quaternion Eigen::Quaterniond in ENU frame.
 * @param enu_vec Eigen::Vector3d ENU vector.
 * @return Eigen::Vector3d FLU vector.
 */
Eigen::Vector3d convertENUtoFLU(const Eigen::Quaterniond &quaternion,
                                const Eigen::Vector3d &enu_vec);

/**
 * @brief Convert FLU (forward, left, up) to ENU (east, north, up) frame.
 *
 * @param quaternion tf2::Quaternion in ENU frame.
 * @param flu_vec Eigen::Vector3d FLU vector.
 * @return Eigen::Vector3d ENU vector.
 */
Eigen::Vector3d convertFLUtoENU(const float roll_angle,
                                const float pitch_angle,
                                const float yaw_angle,
                                const Eigen::Vector3d &flu_vec);

/**
 * @brief Convert FLU (forward, left, up) to ENU (east, north, up) frame.
 *
 * @param quaternion tf2::Quaternion in ENU frame.
 * @param flu_vec Eigen::Vector3d FLU vector.
 * @return Eigen::Vector3d ENU vector.
 */
Eigen::Vector3d convertFLUtoENU(const tf2::Quaternion &quaternion, const Eigen::Vector3d &flu_vec);

/**
 * @brief Convert FLU (forward, left, up) to ENU (east, north, up) frame.
 *
 * @param quaternion geometry_msgs::msg::Quaternion in ENU frame.
 * @param flu_vec Eigen::Vector3d FLU vector.
 * @return Eigen::Vector3d ENU vector.
 */
Eigen::Vector3d convertFLUtoENU(const geometry_msgs::msg::Quaternion &quaternion,
                                const Eigen::Vector3d &flu_vec);

/**
 * @brief Convert FLU (forward, left, up) to ENU (east, north, up) frame.
 *
 * @param quaternion Eigen::Quaterniond in ENU frame.
 * @param flu_vec Eigen::Vector3d FLU vector.
 * @return Eigen::Vector3d ENU vector.
 */
Eigen::Vector3d convertFLUtoENU(const Eigen::Quaterniond &quaternion,
                                const Eigen::Vector3d &flu_vec);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion tf2::Quaternion to convert.
 * @param roll double pointer to store roll angle.
 * @param pitch double pointer to store pitch angle.
 * @param yaw double pointer to store yaw angle.
 */
void quaternionToEuler(const tf2::Quaternion &quaternion, double &roll, double &pitch, double &yaw);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion geometry_msgs::msg::Quaternion to convert.
 * @param roll double pointer to store roll angle.
 * @param pitch double pointer to store pitch angle.
 * @param yaw double pointer to store yaw angle.
 */
void quaternionToEuler(const geometry_msgs::msg::Quaternion &quaternion,
                       double &roll,
                       double &pitch,
                       double &yaw);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion Eigen::Quaternion to convert.
 * @param yaw double pointer to store yaw angle.
 */
void quaternionToEuler(const Eigen::Quaterniond &quaternion,
                       double &roll,
                       double &pitch,
                       double &yaw);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @param roll double roll angle.
 * @param pitch double pitch angle.
 * @param yaw double yaw angle.
 * @param quaternion tf2::Quaternion pointer to store quaternion.
 */
void eulerToQuaternion(const double &roll,
                       const double &pitch,
                       const double &yaw,
                       tf2::Quaternion &quaternion);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @param roll double roll angle.
 * @param pitch double pitch angle.
 * @param yaw double yaw angle.
 * @param quaternion geometry_msgs::msg::Quaternion pointer to store quaternion.
 */
void eulerToQuaternion(const double &roll,
                       const double &pitch,
                       const double &yaw,
                       geometry_msgs::msg::Quaternion &quaternion);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @param roll double roll angle.
 * @param pitch double pitch angle.
 * @param yaw double yaw angle.
 * @param quaternion Eigen::Quaterniond pointer to store quaternion.
 */
void eulerToQuaternion(const double &roll,
                       const double &pitch,
                       const double &yaw,
                       Eigen::Quaterniond &quaternion);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion tf2::Quaternion to convert.
 * @param yaw double pointer to store yaw angle.
 */
double getYawFromQuaternion(const tf2::Quaternion &quaternion);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion geometry_msgs::msg::Quaternion to convert.
 * @param yaw double pointer to store yaw angle.
 * @return Double yaw angle.
 */
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quaternion);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion Eigen::Quaternion to convert.
 * @param yaw double pointer to store yaw angle.
 * @return Double yaw angle.
 */
double getYawFromQuaternion(const Eigen::Quaterniond &quaternion);

/**
 * @brief Compute the angle between of a given vector in 2D and the unitary vector (1,0).
 *
 * @param x double x coordinate of the vector.
 * @param y double y coordinate of the vector.
 * @return Double yaw angle.
 */
double getVector2DAngle(const double &x, const double &y);

};  // namespace frame

};  // namespace as2

#endif  // __AEROSTACK2_CODE_UTILS_HPP__
