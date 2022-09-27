/*!*******************************************************************************************
 *  \file       synchronous_service_client.hpp
 *  \brief      Class for handling synchronous service clients in ROS2
 *              without taking care about the spin() method
 *  \authors    Miguel Fernández Cortizas
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

#ifndef __SYNCHRONOUS_SERVICE_CLIENT_HPP__
#define __SYNCHRONOUS_SERVICE_CLIENT_HPP__

#include <memory>
#include <string>

#include "as2_core/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2 {

template <class ServiceT>
/**
 * @brief Class for handling synchronous service clients in ROS2
 *       without taking care about the spin() method
 */
class SynchronousServiceClient {
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
  std::string service_name_;
  std::string node_name_;

public:
  using SharedPtr = std::shared_ptr<SynchronousServiceClient<ServiceT>>;

  /**
   * @brief Constructor
   * @param service_name Name of the service
   */
  SynchronousServiceClient(std::string service_name) {
    service_name_ = service_name;
    node_name_    = service_name;
    // replace all '/' with '_' in name
    std::replace(node_name_.begin(), node_name_.end(), '/', '_');
  }

  /**
   * @brief Send Request synchronously to the service
   * @param request Request to be sent to the service
   * @param respone Response received from the service
   * @param wait_time Time to wait for the service to be available in seconds
   * @return True if the service was called successfully, false otherwise
   */

  bool sendRequest(const RequestT &req, ResponseT &resp, int wait_time = 0) {
    auto resp_ptr = std::make_shared<ResponseT>(resp);
    if (!sendRequest(std::make_shared<RequestT>(req), resp_ptr, wait_time)) {
      return false;
    }
    resp = *resp_ptr.get();
    return true;
  };

  /**
   * @brief Send Request synchronously to the service
   * @param request Request to be sent to the service
   * @param respone Response received from the service
   * @param wait_time Time to wait for the service to be available in seconds
   * @return True if the service was called successfully, false otherwise
   */
  bool sendRequest(const std::shared_ptr<RequestT> &req,
                   std::shared_ptr<ResponseT> &resp,
                   int wait_time = 0) {
    auto node   = std::make_shared<rclcpp::Node>(node_name_);
    auto client = node->create_client<ServiceT>(service_name_);

    if (wait_time <= 0) {
      while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for the service. exiting.");
          return false;
        }
        RCLCPP_INFO(node->get_logger(), "service: %s not available, waiting again...",
                    service_name_.c_str());
      }
    } else {
      if (!client->wait_for_service(std::chrono::seconds(wait_time))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for the service. exiting.");
          return false;
        }
        RCLCPP_WARN(node->get_logger(), "service: %s not available, returning False ",
                    service_name_.c_str());
        return false;
      }
    }

    auto result = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(1)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "failed to receive response from service '%s'",
                  service_name_.c_str());
      return false;
    }

    resp = result.get();
    return true;
  }  // namespace as2

protected:
};  // namespace as2

}  // namespace as2
#endif
