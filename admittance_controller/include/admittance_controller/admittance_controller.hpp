#pragma once

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <fstream>

// Linear algebra library
#include <armadillo>

// Jacobian
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf/model.h>

// Eigen 
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

namespace admittance_controller
{

template <class SegmentImpl, class HardwareInterface>
class AdmittanceController
  : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
{
public:
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 3, 3> Matrix3d;

  AdmittanceController()
  {
    force_torque_sub_ = nh_.subscribe("/filtered_wrench", 2, &AdmittanceController::forceTorqueCB, this);
    tcp_sub_ = nh_.subscribe("/tf", 2, &AdmittanceController::tcpCB, this);
    current_vf_.fill(0.0);

    urdf::Model robot_model;
    std::string robot_description;
    nh_.getParam("robot_description", robot_description);
    if (!robot_model.initString(robot_description))
    {
      std::cout << "Failled to initialize robot description" << std::endl;
    }
    KDL::Tree robot_tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
    {
      std::cout << "Failed to get robot tree" << std::endl;
    }
    if (!robot_tree.getChain("base", "tool0", robot_chain_)) 
    {
      std::cout << "Failed to get chain" << std::endl;
    }

    
    jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));
    M_.resize(6,6);
    M_(0,0) = 8.0*10e3;
    M_(1,1) = 8.0*10e3;
    M_(2,2) = 8.0*10e3;
    M_(3,3) = 8.0*10e3;
    M_(4,4) = 8.0*10e3;
    M_(5,5) = 8.0*10e3;

    D_.resize(6,6);
    D_(0,0) = 4.0*10e2;
    D_(1,1) = 4.0*10e2;
    D_(2,2) = 4.0*10e2;
    D_(3,3) = 4.0*10e2;
    D_(4,4) = 4.0*10e2;
    D_(5,5) = 4.0*10e2;
    cur_pos_.fill(0.0);
    cur_rot_.fill(0.0);
    current_vf_.fill(0.0);
    desired_force_z_ = 10.0;
    pid_ = control_toolbox::Pid(2.0e-4, 1.8e-4, 0.0, 0.0, -0.0, false);
    current_wrench_.fill(0.0);
    file_.open("/home/mads/git/project_in_advanced_robotics/admittance_controller/joints.csv");
  }

  ~AdmittanceController()
  {
    file_.close();
  }

  // Try to transform the wrench to the base before applying the admittance control, to see if it improves anything
  // It seems to be working very well in simulation, not sure if am willing to test on a real robot.
  void update(const ros::Time& time, const ros::Duration& period)
  {
    
    typename Base::TimeData time_data;
    time_data.time   = time; // Cache current time
    time_data.period = period; // Cache current control period
    time_data.uptime = this->time_data_.readFromRT()->uptime + period; // Update controller uptime
    this->time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

  
    Matrix6d A;
    A.fill(0.0);
    A.topLeftCorner(3,3) = cur_rot_;
    A.bottomRightCorner(3,3) = cur_rot_;
    A.bottomLeftCorner(3,3) = cur_pos_ * cur_rot_;

    Eigen::VectorXd wrench_in_base(6); 
    wrench_in_base[0] = current_wrench_[3];
    wrench_in_base[1] = current_wrench_[4];
    wrench_in_base[2] = current_wrench_[5];
    wrench_in_base[3] = current_wrench_[0];
    wrench_in_base[4] = current_wrench_[1];
    wrench_in_base[5] = current_wrench_[2];

    wrench_in_base = A * wrench_in_base;
    arma::vec6 sensed_wrench_in_base;

    sensed_wrench_in_base[0] = wrench_in_base[3];
    sensed_wrench_in_base[1] = wrench_in_base[4];
    sensed_wrench_in_base[2] = wrench_in_base[5];
    sensed_wrench_in_base[3] = wrench_in_base[0];
    sensed_wrench_in_base[4] = wrench_in_base[1];
    sensed_wrench_in_base[5] = wrench_in_base[2];

    arma::vec6 vf_dot = arma::inv(M_) * (sensed_wrench_in_base - D_ * current_vf_);

    double z_force_error = desired_force_z_ - (current_wrench_[2]*-1.0);
    double force_contribution = pid_.computeCommand(z_force_error, period);
    //std::cout << force_contribution << std::endl;


    // Integrate acceleration
    current_vf_ += vf_dot * period.toSec();
    //current_vf_[2] = current_vf_[2] + force_contribution;
    //std::cout << current_vf_[0] << " " << current_vf_[1] << " " << current_vf_[2] << std::endl;

    Eigen::VectorXd cur_vel(6);

    Eigen::VectorXd cur_vel_in_base(6);


    // Transform velocities to [rotational translational]
    cur_vel << current_vf_[3], current_vf_[4], current_vf_[5], current_vf_[0], current_vf_[1], current_vf_[2];
    cur_vel_in_base = A*cur_vel;


    // Transform current velocities back to [translational rotational] or store base velocity, since the wrench has already been transformed to the base
    Eigen::VectorXd base_vel(6);
    base_vel << cur_vel[0], cur_vel[1], cur_vel[2], cur_vel[3], cur_vel[4], cur_vel[5];


    // Get current joint positions
    KDL::JntArray cur_joints(6);
    KDL::JntArray cur_vel_from_robot(6);
    for (unsigned int i = 0; i < 6; ++i)
    {
      cur_joints(i) = this->joints_[i].getPosition();
      this->desired_state_.position[i] = this->joints_[i].getPosition();
      this->desired_state_.acceleration[i] = 0.0; // Acceleration should just be zero
      // errror is always zero
      this->state_error_.position[i] = 0.0; 
      this->state_error_.velocity[i] = 0.0;
      this->state_error_.acceleration[i] = 0.0; 
      cur_vel_from_robot(i) = this->joints_[i].getVelocity();
    }
    KDL::Jacobian jac(6);
    jac_solver_->JntToJac(cur_joints, jac);

    Eigen::VectorXd joint_vel(6);
    joint_vel = jac.data.inverse() * base_vel;

    //std::cout << cur_joints(0) << " " << cur_joints(1) << " " << cur_joints(2) << std::endl;
    //std::cout << cur_joints(3) << " " << cur_joints(4) << " " << cur_joints(5) << std::endl;
    std::cout << "current joint velocities" << std::endl;
    std::cout << joint_vel(0) << " " << joint_vel(1) << " " << joint_vel(2) << std::endl;
    std::cout << " " << joint_vel(3) << " " << joint_vel(4) << " " << joint_vel(5) << std::endl;

    //std::cout << base_vel(0) <<  " " << base_vel(1) << " " << base_vel(2) << std::endl;
    for (unsigned int i = 0; i < 6; ++i)
    {
      this->desired_state_.velocity[i] = joint_vel(i);
      file_ << joint_vel(i) << ",";      
    }
    file_ << "\n";
  
    this->hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period, this->desired_state_, this->state_error_);
  }

  void forceTorqueCB(const geometry_msgs::WrenchStamped& wrench)
  {
    current_wrench_[0] = wrench.wrench.force.x;
    current_wrench_[1] = wrench.wrench.force.y;
    current_wrench_[2] = wrench.wrench.force.z;
    current_wrench_[3] = wrench.wrench.torque.x;
    current_wrench_[4] = wrench.wrench.torque.y;
    current_wrench_[5] = wrench.wrench.torque.z;
    //current_wrench_[2] = -10.0;
  }

  void tcpCB(const tf2_msgs::TFMessage::ConstPtr& msg)
  {

    rotation_cur_pos_ = KDL::Rotation::Quaternion(msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z, msg->transforms[0].transform.rotation.w);
    cur_rot_ << rotation_cur_pos_.data[0], rotation_cur_pos_.data[1], rotation_cur_pos_.data[2],
                rotation_cur_pos_.data[3], rotation_cur_pos_.data[4], rotation_cur_pos_.data[5],
                rotation_cur_pos_.data[6], rotation_cur_pos_.data[7], rotation_cur_pos_.data[8];
    cur_pos_vec_ = KDL::Vector(msg->transforms[0].transform.translation.x, msg->transforms[0].transform.translation.y, msg->transforms[0].transform.translation.z);
    cur_pos_ << 0.0, -cur_pos_vec_[2], cur_pos_vec_[1],
               cur_pos_vec_[2], 0.0,  -cur_pos_vec_[0],
              -cur_pos_vec_[1], cur_pos_vec_[0], 0.0;
  }

private:
  using Base = joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>;
  ros::Subscriber force_torque_sub_;
  ros::Subscriber tcp_sub_;
  arma::mat M_;
  arma::mat D_;
  arma::vec6 current_wrench_;
  arma::vec6 current_vf_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  ros::NodeHandle nh_;
  KDL::Chain robot_chain_;
  KDL::Rotation rotation_cur_pos_;
  KDL::Vector cur_pos_vec_;
  Matrix3d cur_rot_;
  Matrix3d cur_pos_;
  double desired_force_z_;
  control_toolbox::Pid pid_;
  std::ofstream file_;
};

}