// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <hybrid_force_pos_controller/hybrid_force_pos_controller.hpp>

namespace position_controllers
{
  /**
   * @brief Hybrid force position controller
   */
  using HybridForcePosController = hybrid_controllers::HybridForcePosController<
    hardware_interface::PositionJointInterface>;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::HybridForcePosController, controller_interface::ControllerBase)