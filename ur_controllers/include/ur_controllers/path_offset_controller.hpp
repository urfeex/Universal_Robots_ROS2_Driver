// Copyright 2026, Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner feex@universal-robots.com
 * \date    2026-03-29
 */
//----------------------------------------------------------------------

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ur_msgs/srv/set_force_mode.hpp>

namespace ur_controllers
{
class PathOffsetController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

private:
  // We misuse SetForceMode and std_srvs/srv/setBOol to make this PoC working. Change this!
  rclcpp::Service<ur_msgs::srv::SetForceMode>::SharedPtr set_path_offset_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;

  realtime_tools::RealtimeThreadSafeBox<double> enabled_buffer_;
  realtime_tools::RealtimeThreadSafeBox<std::array<double, 6>> path_offset_buffer_;
  realtime_tools::RealtimeThreadSafeBox<double> path_offset_type_buffer_;

  std::atomic<bool> enabled_changed_ = false;
  std::atomic<bool> path_offset_changed_ = false;

  bool setEnabled(const std_srvs::srv::SetBool::Request::SharedPtr request,
                  std_srvs::srv::SetBool::Response::SharedPtr response);
  bool setPathOffset(const ur_msgs::srv::SetForceMode::Request::SharedPtr request,
                     ur_msgs::srv::SetForceMode::Response::SharedPtr response);
};
}  // namespace ur_controllers
