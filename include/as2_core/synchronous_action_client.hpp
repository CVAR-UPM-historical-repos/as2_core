/*!*******************************************************************************************
 *  \file       synchronous_action_client.hpp
 *  \brief      Class for handling synchronous service clients in ROS2
 *              without taking care about the spin() method
 *  \authors    Rafael Pérez Seguí

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

#ifndef __SYNCHRONOUS_ACTION_CLIENT_HPP__
#define __SYNCHRONOUS_ACTION_CLIENT_HPP__

#include <memory>
#include <string>

#include "as2_core/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2 {

template <class ActionT>
/**
 * @brief Class for handling synchronous service clients in ROS2
 *       without taking care about the spin() method
 */
class SynchronousActionClient {
  typedef typename ActionT::Goal GoalT;
  std::string action_name_;
  as2::Node *node_;

public:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

public:
  using SharedPtr = std::shared_ptr<SynchronousActionClient<ActionT>>;

  /**
   * @brief Constructor
   * @param service_name Name of the service
   */
  SynchronousActionClient(std::string action_name_, as2::Node *node)
      : action_name_(action_name_), node_(node) {
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name_, callback_group_);
  }

  bool wait_for_action_server(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(2)) {
    return action_client_->wait_for_action_server(timeout);
  }

  /**
   * @brief Sends a goal to the action server and waits for the goal acceptance
   * @param goal Goal to be sent
   * @return True if the goal was accepted, false otherwise
   */
  bool send_goal(GoalT goal) {
    auto send_goal_options           = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.callback_group = callback_group_;
    auto future_goal_handle          = action_client_->async_send_goal(goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }
    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      return false;
    }
    return true;
  }

protected:
};  // namespace as2

}  // namespace as2
#endif
