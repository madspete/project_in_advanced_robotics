#include <ros/ros.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Trigger.h>

// Forward kinematics
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kinfam.hpp>
#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include <kdl/framevel.hpp>

#include "trajectory_learning/gmm_gmr_joint.hpp"

// Implemented controller
#include <geometry_msgs/WrenchStamped.h>
#include <hybrid_force_pos_controller/targetTraj.h>

#include <geometry_msgs/PoseStamped.h>
// Make a function to switch on the correct controllers - maybe this is needed

int main(int argc, char* argv[])
{

  // This must be called before anything else ROS-related
  ros::init(argc, argv, "gmm_joint");
  ros::NodeHandle nh;

  // Align the demonstrated trajectories
  trajectory_learning::GMMAndGMRJoint gmm_gmr("/home/marcus/project/project_in_advanced_robotics/trajectory_learning/demonstrations_joint");

  gmm_gmr.gmm_learn();
  std::vector<unsigned int > in = {6};
  std::vector<unsigned int > out = {0,1,2,3,4,5};
  std::vector<double> input_data;

  double step = 0.002; //This can be adjusted to what is needed.
  std::ofstream time_file;
  time_file.open("/home/mads/git/project_in_advanced_robotics/input_time.csv");
  for (double i = step; i < gmm_gmr.get_end_time(); i = i +step)
  {
    std::cout << "input time" << std::endl;
    input_data.push_back(i);
    time_file << i << "\n";
  }
  time_file.close();
  std::vector<Vector> target;
  gmm_gmr.gmr_calculation_fast_gmm(target, input_data, in, out);


  // Forward kinematics
  urdf::Model robot_model;
  std::string robot_description;
  nh.getParam("robot_description", robot_description);
  if (!robot_model.initString(robot_description))
  {
    std::cout << "Failled to initialize robot description" << std::endl;
    return 0;
  }
  KDL::Tree robot_tree;
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    std::cout << "Failed to get robot tree" << std::endl;
    return 0;
  }
  KDL::Chain robot_chain;
  if (!robot_tree.getChain("base", "tool0", robot_chain)) 
  {
    std::cout << "Failed to get chain" << std::endl;
    return 0;
  }
  KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
  
  // First point in the trajectory cannot be 0.0 time, it should be a time different from zero.
  cartesian_control_msgs::CartesianTrajectory traj;
  cartesian_control_msgs::CartesianTrajectory first_goal;
  double cur_time = 0.01;
  std::ofstream myfile;
  std::vector<geometry_msgs::WrenchStamped> wrench;
  geometry_msgs::WrenchStamped cur_wrench;
  cur_wrench.wrench.force.x = 0.0;
  cur_wrench.wrench.force.y = 0.0;
  cur_wrench.wrench.force.z = 8.0; // Seems to be working
  cur_wrench.wrench.torque.x = 0.0;
  cur_wrench.wrench.torque.y = 0.0;
  cur_wrench.wrench.torque.z = 0.0;
  myfile.open ("/home/marcus/project/project_in_advanced_robotics/trajectory_learning/traj_tcp.csv");
  step = 0.01;
  for (unsigned int i = 0; i < target.size(); ++i)
  {
    KDL::JntArray joints(6);
    KDL::Frame pose;
    
    // The UR publishes joint angles in this order [elbow shoulder_lift shoulder_pan wrist1 2 3]
    joints(0) = target[i][2];
    joints(1) = target[i][1];
    joints(2) = target[i][0];
    joints(3) = target[i][3];
    joints(4) = target[i][4];
    joints(5) = target[i][5];

    fk_solver.JntToCart(joints, pose);

    // Pose
    ros_controllers_cartesian::CartesianState state;
    state.p.x() = pose.p.x();
    state.p.y() = pose.p.y();
    state.p.z() = pose.p.z();
    pose.M.GetQuaternion(state.q.x(), state.q.y(), state.q.z(), state.q.w());
    cartesian_control_msgs::CartesianTrajectoryPoint cur_point;
    ros::Duration dur(cur_time);
    cur_point.time_from_start = dur;
    cur_point.pose = state.toMsg().pose;
    traj.points.push_back(cur_point);
    wrench.push_back(cur_wrench);
    cur_time = cur_time + step;
    if (i == 0)
    {
      cur_point.time_from_start = ros::Duration(5.0);
      first_goal.points.push_back(cur_point);
      std::cout <<  target[i][0] * (180.0/PI) << "," << target[i][1] * (180.0/PI) << "," << target[i][2] * (180.0/PI) << "," << target[i][3] * (180.0/PI) << "," << target[i][4] * (180.0/PI) << "," << target[i][5] * (180.0/PI) << std::endl;
    }
      // std::cout <<  target[i][0] << "," << target[i][1]  << "," << target[i][2] << "," << target[i][3]  << "," << target[i][4] << "," << target[i][5] << std::endl;

    //myfile << 100 << "," << 200 << "," << state.p.x() << "," << state.p.y() << "," << state.p.z() << "," << state.q.w() << "," << state.q.x() << "," << state.q.y() << "," << state.q.z() << "\n";
    myfile << target[i][0] << "," << target[i][1] << "," << target[i][2] << "," << target[i][3] << "," << target[i][4] << "," << target[i][5] << "\n";
  }

  myfile.close();

  gmm_gmr.write_data_to_file("/home/marcus/project/project_in_advanced_robotics/trajectory_learning/gmm_model/means.txt", 
                             "/home/marcus/project/project_in_advanced_robotics/trajectory_learning/gmm_model/cov.txt",
                             "/home/marcus/project/project_in_advanced_robotics/trajectory_learning/gmm_model/priors.txt");

  ros::ServiceClient srv_client = nh.serviceClient<hybrid_force_pos_controller::targetTraj>("target_hybrid");

  hybrid_force_pos_controller::targetTraj srv;
  std_srvs::Trigger std_trigger;

  srv.request.traj = traj;
  srv.request.wrenches = wrench;
  srv_client.call(srv);

  std::cout << "service has been called" << std::endl;
}