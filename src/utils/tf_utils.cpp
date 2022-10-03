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

#include "as2_core/utils/tf_utils.hpp"
#include <tf2/convert.h>

namespace as2 {
namespace tf {

std::string generateTfName(rclcpp::Node *node, std::string _frame_name) {
  return generateTfName(node->get_namespace(), _frame_name);
}

std::string generateTfName(const std::string &_namespace, const std::string &_frame_name) {
  if (!_frame_name.size()) {
    throw std::runtime_error("Empty frame name");
  }
  if (_frame_name[0] == '/') {
    return _frame_name.substr(1);
  }
  if (!_namespace.size()) {
    RCLCPP_WARN(rclcpp::get_logger("tf_utils"),
                "The frame name [%s] is not absolute and the node namespace is empty. This could "
                "lead to conflicts.",
                _frame_name.c_str());
    return _frame_name;
  }
  if (_namespace[0] == '/') {
    return _namespace.substr(1) + "/" + _frame_name;
  }
  return _namespace + "/" + _frame_name;
}

geometry_msgs::msg::TransformStamped getTransformation(const std::string &_frame_id,
                                                       const std::string &_child_frame_id,
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

geometry_msgs::msg::PointStamped TfHandler::convert(const geometry_msgs::msg::PointStamped &_point,
                                                    const std::string &target_frame) {
  geometry_msgs::msg::PointStamped point_out;
  const auto frame = formatFrameId(target_frame);
  tf2::doTransform(_point, point_out,
                   tf_buffer_->lookupTransform(frame, _point.header.frame_id, _point.header.stamp));
  point_out.header.stamp    = _point.header.stamp;
  point_out.header.frame_id = frame;
  return point_out;
};

geometry_msgs::msg::PoseStamped TfHandler::convert(const geometry_msgs::msg::PoseStamped &_pose,
                                                   const std::string &target_frame) {
  geometry_msgs::msg::PoseStamped pose_out;
  const auto frame = formatFrameId(target_frame);
  tf2::doTransform(_pose, pose_out,
                   tf_buffer_->lookupTransform(frame, _pose.header.frame_id, _pose.header.stamp));
  pose_out.header.frame_id = frame;
  pose_out.header.stamp    = _pose.header.stamp;
  return pose_out;
};

geometry_msgs::msg::TwistStamped TfHandler::convert(const geometry_msgs::msg::TwistStamped &_twist,
                                                    const std::string &target_frame) {
  geometry_msgs::msg::TwistStamped twist_out;
  geometry_msgs::msg::Vector3Stamped vector_out;

  vector_out.header = _twist.header;
  vector_out.vector = _twist.twist.linear;
  // transform linear speed
  vector_out              = convert(vector_out, target_frame);
  twist_out.header        = vector_out.header;
  twist_out.twist.linear  = vector_out.vector;
  twist_out.twist.angular = _twist.twist.angular;
  return twist_out;
};

geometry_msgs::msg::Vector3Stamped TfHandler::convert(
    const geometry_msgs::msg::Vector3Stamped &_vector,
    const std::string &target_frame) {
  geometry_msgs::msg::Vector3Stamped vector_out;
  const auto frame = formatFrameId(target_frame);
  tf2::doTransform(
      _vector, vector_out,
      tf_buffer_->lookupTransform(frame, _vector.header.frame_id, _vector.header.stamp));
  vector_out.header.frame_id = frame;
  vector_out.header.stamp    = _vector.header.stamp;
  return vector_out;
};

nav_msgs::msg::Path TfHandler::convert(const nav_msgs::msg::Path &_path,
                                       const std::string &target_frame) {
  nav_msgs::msg::Path path_out;
  const auto frame = formatFrameId(target_frame);
  for (auto &pose : _path.poses) {
    geometry_msgs::msg::PoseStamped pose_out;
    tf2::doTransform(pose, pose_out,
                     tf_buffer_->lookupTransform(frame, pose.header.frame_id, pose.header.stamp));
    path_out.poses.push_back(pose_out);
  }
  path_out.header.frame_id = frame;
  path_out.header.stamp    = _path.header.stamp;
  return path_out;
};

geometry_msgs::msg::PoseStamped TfHandler::getPoseStamped(const std::string &target_frame,
                                                          const std::string &source_frame,
                                                          const tf2::TimePoint &time) {
  auto transform =
      tf_buffer_->lookupTransform(formatFrameId(target_frame), formatFrameId(source_frame), time);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id    = target_frame;
  pose.header.stamp       = transform.header.stamp;
  pose.pose.position.x    = transform.transform.translation.x;
  pose.pose.position.y    = transform.transform.translation.y;
  pose.pose.position.z    = transform.transform.translation.z;
  pose.pose.orientation.x = transform.transform.rotation.x;
  pose.pose.orientation.y = transform.transform.rotation.y;
  pose.pose.orientation.z = transform.transform.rotation.z;
  pose.pose.orientation.w = transform.transform.rotation.w;
  return pose;
};

geometry_msgs::msg::TransformStamped TfHandler::getTransform(const std::string &target_frame,
                                                             const std::string &source_frame,
                                                             const tf2::TimePoint &time) {
  return tf_buffer_->lookupTransform(formatFrameId(target_frame), formatFrameId(source_frame),
                                     time);
};
}  // namespace tf
}  // namespace as2
