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

#include "ur_controllers/path_offset_controller.hpp"

namespace ur_controllers
{
PathOffsetController::CallbackReturn PathOffsetController::on_init()
{
  // potentially get parameters here in the future, but for now there are none
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PathOffsetController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back("path_offset/path_offset_x");
  config.names.emplace_back("path_offset/path_offset_y");
  config.names.emplace_back("path_offset/path_offset_z");
  config.names.emplace_back("path_offset/path_offset_rx");
  config.names.emplace_back("path_offset/path_offset_ry");
  config.names.emplace_back("path_offset/path_offset_rz");
  config.names.emplace_back("path_offset/path_offset_type");
  config.names.emplace_back("path_offset/path_offset_enable");
  config.names.emplace_back("path_offset/async_success");
  return config;
}

controller_interface::InterfaceConfiguration PathOffsetController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type PathOffsetController::update(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/)
{
  if (enabled_changed_) {
    std::ignore = command_interfaces_[7].set_value(enabled_buffer_.get());
    enabled_changed_ = false;
  }

  if (path_offset_changed_) {
    const auto path_offset = path_offset_buffer_.get();
    std::ignore = command_interfaces_[0].set_value(path_offset[0]);
    std::ignore = command_interfaces_[1].set_value(path_offset[1]);
    std::ignore = command_interfaces_[2].set_value(path_offset[2]);
    std::ignore = command_interfaces_[3].set_value(path_offset[3]);
    std::ignore = command_interfaces_[4].set_value(path_offset[4]);
    std::ignore = command_interfaces_[5].set_value(path_offset[5]);
    std::ignore = command_interfaces_[6].set_value(path_offset_type_buffer_.get());
    path_offset_changed_ = false;
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
PathOffsetController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto logger = get_node()->get_logger();
  enable_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
      "~/set_enabled",
      std::bind(&PathOffsetController::setEnabled, this, std::placeholders::_1, std::placeholders::_2));
  set_path_offset_srv_ = get_node()->create_service<ur_msgs::srv::SetForceMode>(
      "~/set_path_offset",
      std::bind(&PathOffsetController::setPathOffset, this, std::placeholders::_1, std::placeholders::_2));
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PathOffsetController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  enabled_changed_ = false;
  path_offset_changed_ = false;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PathOffsetController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PathOffsetController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  enable_srv_.reset();
  set_path_offset_srv_.reset();
  return CallbackReturn::SUCCESS;
}

bool PathOffsetController::setEnabled(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                      std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (request->data) {
    RCLCPP_INFO(get_node()->get_logger(), "Enabling path offset.");
    enabled_buffer_.set(1.0);
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Disabling path offset.");
    enabled_buffer_.set(0.0);
  }
  enabled_changed_ = true;
  response->success = true;
  return true;
}

bool PathOffsetController::setPathOffset(const ur_msgs::srv::SetForceMode::Request::SharedPtr request,
                                         ur_msgs::srv::SetForceMode::Response::SharedPtr response)
{
  std::array<double, 6> path_offset{ request->task_frame.pose.position.x,    request->task_frame.pose.position.y,
                                     request->task_frame.pose.position.z,    request->task_frame.pose.orientation.x,
                                     request->task_frame.pose.orientation.y, request->task_frame.pose.orientation.z };

  path_offset_buffer_.set(path_offset);

  path_offset_type_buffer_.set(static_cast<double>(request->type));
  RCLCPP_INFO(get_node()->get_logger(), "Setting path offset to: [%f, %f, %f, %f, %f, %f] with type %d", path_offset[0],
              path_offset[1], path_offset[2], path_offset[3], path_offset[4], path_offset[5], request->type);

  path_offset_changed_ = true;

  response->success = true;
  return true;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::PathOffsetController, controller_interface::ControllerInterface)
