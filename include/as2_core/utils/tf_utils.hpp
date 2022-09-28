/*!*******************************************************************************************
 *  \file       tf_utils.hpp
 *  \brief      Tranform utilities library header file.
 *  \authors    David Perez Saura
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

#ifndef TF_UTILS_HPP_
#define TF_UTILS_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace as2 {
namespace tf {

/**
 * @brief Add namespace to the name of the Transform frame id
 *
 * @param _namespace
 * @param _frame_name
 * @return std::string namespace/frame_id
 */
std::string generateTfName(std::string _namespace, std::string _frame_name);

/**
 * @brief Generate a Transform message from translation and rotation in Euler angles
 *
 * @param _frame_id
 * @param _child_frame_id
 * @param _translation_x
 * @param _translation_y
 * @param _translation_z
 * @param _roll
 * @param _pitch
 * @param _yaw
 * @return geometry_msgs::msg::TransformStamped
 */
geometry_msgs::msg::TransformStamped getTransformation(const std::string &_frame_id,
                                                       const std::string &_child_frame_id,
                                                       double _translation_x,
                                                       double _translation_y,
                                                       double _translation_z,
                                                       double _roll,
                                                       double _pitch,
                                                       double _yaw);

}  // namespace tf
}  // namespace as2
#endif  // TF_UTILS_HPP_
