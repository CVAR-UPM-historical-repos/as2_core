/*!*******************************************************************************************
 *  \file       tf_utils.cpp
 *  \brief      Tranform utilities library implementation file.
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

#include "tf_utils.hpp"

// TODO: add namespace and documentation for this library, move this file inside a folder called
// frame_utils

std::string generateTfName(std::string _namespace, std::string _frame_name) {
  std::string tf_name;

  if (_namespace.find("/") == 0) {
    _namespace = _namespace.substr(1);
  }
  if (_frame_name.find("/") == 0) {
    _frame_name = _frame_name.substr(1);
  }

  if (_namespace != "") {
    return _namespace + "/" + _frame_name;
  } else {
    return _frame_name;
  }
}

geometry_msgs::msg::TransformStamped getTransformation(const std::string& _frame_id,
                                                       const std::string& _child_frame_id,
                                                       double _translation_x,
                                                       double _translation_y,
                                                       double _translation_z,
                                                       double _roll,
                                                       double _pitch,
                                                       double _yaw) {
  geometry_msgs::msg::TransformStamped transformation;

  transformation.header.frame_id         = _frame_id;
  transformation.child_frame_id          = _child_frame_id;
  transformation.transform.translation.x = _translation_x;
  transformation.transform.translation.y = _translation_y;
  transformation.transform.translation.z = _translation_z;
  tf2::Quaternion q;
  q.setRPY(_roll, _pitch, _yaw);
  transformation.transform.rotation.x = q.x();
  transformation.transform.rotation.y = q.y();
  transformation.transform.rotation.z = q.z();
  transformation.transform.rotation.w = q.w();

  return transformation;
}
