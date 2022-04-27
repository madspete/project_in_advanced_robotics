// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
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

# pragma once

// Controller dependencies
#include <cartesian_trajectory_controller/cartesian_trajectory_controller.h>

// Services
#include <hybrid_force_pos_controller/targetTraj.h>
#include <std_srvs/Trigger.h>
#include <ur_dashboard_msgs/AddToLog.h>

// Messages
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

// Ros
#include <ros/ros.h>
#include <control_toolbox/pid.h>

// Filter (It only works with absolute path for some reason)
#include <../../robot_listener/include/robot_listener/butterworth_filter.hpp>

// Other
#include <vector>
#include <atomic>
#include <limits>
#include <deque>

// Kinematics
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace hybrid_controllers
{

template <class HardwareInterface>
class HybridForcePosController 
  : public cartesian_trajectory_controller::CartesianTrajectoryController<HardwareInterface>
{
public:  
  HybridForcePosController() :  cartesian_trajectory_controller::CartesianTrajectoryController<HardwareInterface>()
  {
    srv_traj_ = nh_.advertiseService("target_hybrid", &HybridForcePosController::trajCB, this);
    force_torque_sub_ = nh_.subscribe("/filtered_wrench", 2, &HybridForcePosController::forceTorqueCB, this);
    tcp_sub_ = nh_.subscribe("/tf", 2, &HybridForcePosController::tcpCB, this);
    srv_start_wrench_ = nh_.serviceClient<ur_dashboard_msgs::AddToLog>("robot_listener/start_wrench_recording");
    srv_stop_wrench_ = nh_.serviceClient<std_srvs::Trigger>("robot_listener/stop_wrench_recording");
    srv_start_tcp_ = nh_.serviceClient<ur_dashboard_msgs::AddToLog>("robot_listener/start_recording");
    srv_stop_tcp_ = nh_.serviceClient<std_srvs::Trigger>("robot_listener/stop_recording");
    current_wrench_.resize(6); // Number of wrenches
    executing_traj_ = false;
    // TODO make this inputs
    // These gains seems to work pretty well 
    // Dgain 8.0e-7
    // pid_ = control_toolbox::Pid(2.35e-4, 6.0e-4, 0.0, 0.0, -0.0, false);
    pid_ = control_toolbox::Pid(1.2e-4, 6.0e-4, 9.8e-7, 0.0, -0.0, false);
    // This works for a line 1.2e-4 3e-4
    // This works for a circle 1.7e-4 3e-4
    // This works above the special 3D printed 1.6e-5, 1.0e-7, 3.1e-7

    // Setup filter TODO makes this as input?
	  double fps = 500, fc = 4.1;
    double FrequencyBands[2] = { 0.000015,fc/(fps/2) };
	  int FiltOrd = 3;

	  filter_a_ = filter_.ComputeDenCoeffs(FiltOrd, FrequencyBands[0], FrequencyBands[1]);
	  filter_b_ = filter_.ComputeNumCoeffs(FiltOrd, FrequencyBands[0], FrequencyBands[1], filter_a_);
  }

  ~HybridForcePosController()
  {
    std_srvs::Trigger srv_stop;
    srv_stop_wrench_.call(srv_stop);
    srv_stop_tcp_.call(srv_stop);
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    if (executing_traj_ == true)
    {
      double old_time = cur_time_.toSec();
      cur_time_ += period;
      if (cur_time_ <= traj_end_time_)
      {
        unsigned int force_index;
        for (unsigned int i = 0; i < traj_point_times_.size()-1; ++i)
        {
          if (cur_time_ <= traj_point_times_[0])
          {
            force_index = 0;
            break;
          }
          else if (traj_point_times_[i] < cur_time_ && cur_time_ <= traj_point_times_[i+1])
          {
            force_index = i;
            break;
          }
        }
        // Calculate force term contribution
        double z_force = wrench_traj_[force_index].wrench.force.z;
        double z_force_error = z_force - (current_wrench_[2]*-1.0);

        // std::cout << " z_force " <<  z_force << "  z_force_error  " <<  z_force_error << std::endl;

        const double force_command = pid_.computeCommand(z_force_error, period);

        // std::cout << " z_force " <<  z_force << "  z_force_error  " <<  z_force_error << " force_command " << force_command << std::endl;

        // sample trajectory
        ros_controllers_cartesian::CartesianState desired;
        pos_traj_.sample(cur_time_.toSec(), desired);



        /*tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        geometry_msgs::TransformStamped tfStamped;
        try
        {
          tfStamped = tfBuffer.lookupTransform("tool0_controller","base",ros::Time(0));
          std::cout << tfStamped << std::endl;
        }
        catch(const std::exception& e)
        {
          std::cerr << "could not get transform" << e.what() << std::endl;
        }
        
        tfStamped.transform.translation.z += force_command;*/

        KDL::Rotation rotation = KDL::Rotation::Quaternion(desired.toMsg().pose.orientation.x, desired.toMsg().pose.orientation.y, desired.toMsg().pose.orientation.z, desired.toMsg().pose.orientation.w);
        KDL::Vector pos_vec(desired.toMsg().pose.position.x, desired.toMsg().pose.position.y, desired.toMsg().pose.position.z);
        KDL::Frame transform_kdl(rotation, pos_vec);


        KDL::Frame tcp_pos;
        tcp_pos.p.data[2] = force_command;
        KDL::Frame new_pos = transform_kdl * tcp_pos;

        // KDL::Frame desired_diff_frame = transform_kdl*init_frame_.Inverse();
        // KDL::Frame cur_diff_frame = cur_pose*init_frame_.Inverse();
        // //std::cout << desired_diff_frame.p.data[0] << " " << desired_diff_frame.p.data[1] << " " << desired_diff_frame.p.data[2] << std::endl;  
        // //std::cout << cur_diff_frame.p.data[0] << " " << cur_diff_frame.p.data[1] << " " << cur_diff_frame.p.data[2] << std::endl;  

        // desired_diff_frame.p.data[2] = cur_diff_frame.p.data[2];

        // KDL::Frame new_pos =  desired_diff_frame * init_frame_;
        std::cout << "new " << new_pos.p.data[0] << " " << new_pos.p.data[1] << " " << new_pos.p.data[2] << std::endl;
        std::cout <<  "des " << desired.p.x() << " " << desired.p.y() << " " << desired.p.z() << std::endl;
        // std::cout << tcp_pos.p.data[0] << " " << tcp_pos.p.data[1] << " " << tcp_pos.p.data[2] << std::endl;

        // Update position with force contribution
        desired.p.x() = new_pos.p.data[0];
        desired.p.y() = new_pos.p.data[1];
        desired.p.z() = new_pos.p.data[2]; 
        new_pos.M.GetQuaternion(desired.q.x(), desired.q.y(), desired.q.z(), desired.q.w());
        // std::cout << "force " << force_command << " target force " <<  z_force << " current force: " << current_wrench_[2] <<  std::endl;
        //std::cout << "x: " << desired.p.x() << " y: " << desired.p.y() << " z: " << desired.p.z() << std::endl;

        // add this to force distribution
        ControlPolicy::updateCommand(desired);
      }
      else
      {
        std_srvs::Trigger srv_stop;
        srv_stop_wrench_.call(srv_stop);
        srv_stop_tcp_.call(srv_stop);
        std::cout << "done executing trajectory, did the robot behave as expected??" << std::endl;
        executing_traj_ = false;
      }      
    }
  }

  void forceTorqueCB(const geometry_msgs::WrenchStamped& wrench)
  {
    /*if (force_z_.size() >= 1000)
    {
      force_z_.pop_front();
    }
    force_z_.push_back(wrench.wrench.force.z);
  
    std::vector<double> z = filter_.filter(force_z_, filter_b_, filter_a_);*/
    current_wrench_[0] = wrench.wrench.force.x;
    current_wrench_[1] = wrench.wrench.force.y;
    current_wrench_[2] = wrench.wrench.force.z; // last elem will be the latest reading
    current_wrench_[3] = wrench.wrench.torque.x;
    current_wrench_[4] = wrench.wrench.torque.y;
    current_wrench_[5] = wrench.wrench.torque.z;
  }

  void tcpCB(const tf2_msgs::TFMessage::ConstPtr& msg)
  {
    if (msg->transforms.size() == 1)
    {
      cur_z_ = -12.0;
      KDL::Rotation rotation_cur_pos = KDL::Rotation::Quaternion(msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z, msg->transforms[0].transform.rotation.w);
      KDL::Vector cur_pos_vec(msg->transforms[0].transform.translation.x, msg->transforms[0].transform.translation.y, msg->transforms[0].transform.translation.z);
      cur_pose_ = KDL::Frame(rotation_cur_pos, cur_pos_vec);
    } 
  }


  bool trajCB(hybrid_force_pos_controller::targetTraj::Request &req, hybrid_force_pos_controller::targetTraj::Response &res)
  {
    try
    {
      if (executing_traj_ == true)
      {
        std::cout << "currently executing trajectory unable to accept a new one" << std::endl;
        return false;
      }
      cartesian_control_msgs::CartesianTrajectory traj_msg = req.traj;
      cartesian_control_msgs::CartesianTrajectory tcp_traj_msg;
      wrench_traj_ = req.wrenches;


      

      auto state = ControlPolicy::getState();
      traj_msg.points.insert(traj_msg.points.begin(), state.toMsg(0));
      pos_traj_.init(traj_msg);
      traj_point_times_.clear();
      for (unsigned int i = 0; i < traj_msg.points.size(); ++i)
      {
        traj_point_times_.push_back(traj_msg.points[i].time_from_start);
      }
      cur_time_ = ros::Duration(0.0);
      cur_force_time_ = ros::Duration(0.0);
      traj_end_time_ = traj_point_times_.back();
      executing_traj_ = true;
      pid_.reset();


      KDL::Rotation rotation = KDL::Rotation::Quaternion(traj_msg.points[0].pose.orientation.x, traj_msg.points[0].pose.orientation.y, traj_msg.points[0].pose.orientation.z, traj_msg.points[0].pose.orientation.w);
      KDL::Vector pos_vec(traj_msg.points[0].pose.position.x, traj_msg.points[0].pose.position.y, traj_msg.points[0].pose.position.z);
      init_frame_ = KDL::Frame(rotation, pos_vec);



      ur_dashboard_msgs::AddToLog srv_start;
      srv_start.request.message = "";
      srv_start_wrench_.call(srv_start);
      srv_start_tcp_.call(srv_start);

      return true;
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }
  }

private:
  using ControlPolicy = ros_controllers_cartesian::ControlPolicy<HardwareInterface>;
  ros::ServiceServer srv_traj_;
  ros::Subscriber force_torque_sub_;
  ros::Subscriber tcp_sub_;
  ros::NodeHandle nh_;
  std::vector<double> current_wrench_;
  ros_controllers_cartesian::CartesianTrajectory pos_traj_;
  std::vector<geometry_msgs::WrenchStamped> wrench_traj_;
  std::atomic<bool> executing_traj_;
  std::vector<ros::Duration> traj_point_times_;
  ros::Duration cur_time_;
  ros::Duration traj_end_time_;
  ros::Duration cur_force_time_;
  ros::ServiceClient srv_start_wrench_;
  ros::ServiceClient srv_stop_wrench_;
  ros::ServiceClient srv_start_tcp_;
  ros::ServiceClient srv_stop_tcp_;

  KDL::Frame init_frame_;

  control_toolbox::Pid pid_;

  // Filter
  ButterworthFilter filter_;
  std::vector<double> filter_a_;
  std::vector<double> filter_b_;
  std::deque<double> force_z_;
  KDL::Frame cur_pose_;

  double cur_z_;
};

}
