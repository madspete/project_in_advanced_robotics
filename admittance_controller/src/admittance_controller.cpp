
// Pluginlib
#include <pluginlib/class_list_macros.h>
#include <trajectory_interface/quintic_spline_segment.h>

// Project
#include <admittance_controller/admittance_controller.hpp>

namespace velocity_controllers
{
  /**
   * @brief Hybrid force position controller
   */
  typedef admittance_controller::AdmittanceController<trajectory_interface::QuinticSplineSegment<double>,
                                                       hardware_interface::VelocityJointInterface>
    admittance_controller;
}

PLUGINLIB_EXPORT_CLASS(velocity_controllers::admittance_controller, controller_interface::ControllerBase)