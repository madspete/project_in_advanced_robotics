#include "trajectory_learning/gmm_gmr.hpp"

#include <ros/ros.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Trigger.h>

// Implemented controller
#include <geometry_msgs/WrenchStamped.h>
#include <hybrid_force_pos_controller/targetTraj.h>

// Make a function to switch on the correct controllers - maybe this is needed

int main(int argc, char* argv[])
{

  // This must be called before anything else ROS-related
  ros::init(argc, argv, "gmm");
  ros::NodeHandle nh;

  // Align the demonstrated trajectories
  trajectory_learning::GMMAndGMR gmm_gmr("/home/marcus/project/project_in_advanced_robotics/trajectory_learning/demonstrations");

  gmm_gmr.gmm_learn();
  std::vector<unsigned int > in = {7};
  std::vector<unsigned int > out = {0,1,2,3,4,5,6};
  std::vector<double> input_data;

  double step = 0.002; //This can be adjusted to what is needed.
  for (double i = step; i < gmm_gmr.get_end_time(); i = i +step)
  {
    input_data.push_back(i);
  }
  std::vector<Vector> target;
  gmm_gmr.gmr_calculation_fast_gmm(target, input_data, in, out);

  
  // First point in the trajectory cannot be 0.0 time, it should be a time different from zero.
  cartesian_control_msgs::CartesianTrajectory traj;
  cartesian_control_msgs::CartesianTrajectory first_goal;
  double cur_time = 0.01;
  std::ofstream myfile;
  std::vector<geometry_msgs::WrenchStamped> wrench;
  geometry_msgs::WrenchStamped cur_wrench;
  cur_wrench.wrench.force.x = 0.0;
  cur_wrench.wrench.force.y = 0.0;
  cur_wrench.wrench.force.z = -12.0; // Seems to be working
  cur_wrench.wrench.torque.x = 0.0;
  cur_wrench.wrench.torque.y = 0.0;
  cur_wrench.wrench.torque.z = 0.0;
  myfile.open ("/home/mads/git/project_in_advanced_robotics/trajectory_learning/traj.csv");
  std::cout << target.size() << " and the other size " << input_data.size() << std::endl;
  for (unsigned int i = 0; i < target.size(); ++i)
  {
    cartesian_control_msgs::CartesianTrajectoryPoint cur_point;
    ros::Duration dur(input_data[i]);
    cur_point.time_from_start = dur;
    cur_point.pose.position.x = target[i][0];
    cur_point.pose.position.y = target[i][1];
    cur_point.pose.position.z = target[i][2];
    cur_point.pose.orientation.w = target[i][3];
    cur_point.pose.orientation.x = target[i][4];
    cur_point.pose.orientation.y = target[i][5];
    cur_point.pose.orientation.z = target[i][6];
    traj.points.push_back(cur_point);
    wrench.push_back(cur_wrench);
    if (i == 0)
    {
      cur_point.time_from_start = ros::Duration(5.0);
      first_goal.points.push_back(cur_point);
      std::cout << target[i][0] << " " << target[i][1] << " " << target[i][2] << " " << std::endl;
      std::cout << target[i][3] << " " << target[i][4] << " " << target[i][5] << " " << target[i][6] << std::endl;
    }
    myfile << cur_point.pose.position.x << "," << cur_point.pose.position.y << "," << cur_point.pose.position.z << "," << cur_point.pose.orientation.w  << "," << cur_point.pose.orientation.x << "," << cur_point.pose.orientation.y << "," << cur_point.pose.orientation.z << "\n";
  }

  myfile.close();

  gmm_gmr.write_data_to_file("/home/marcus/project/project_in_advanced_robotics/trajectory_learning/gmm_model/means.txt", 
                             "/home/marcus/project/project_in_advanced_robotics/trajectory_learning/gmm_model/cov.txt",
                             "/home/marcus/project/project_in_advanced_robotics/trajectory_learning/gmm_model/priors.txt");

  ros::ServiceClient srv_client = nh.serviceClient<hybrid_force_pos_controller::targetTraj>("target_hybrid");
  ros::ServiceClient srv_zero_ft = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/zero_ftsensor");
  
  hybrid_force_pos_controller::targetTraj srv;
  std_srvs::Trigger std_trigger;

  // When starting the program it will zero the ft sensor
  //srv_zero_ft.call(std_trigger);

  srv.request.traj = traj;
  srv.request.wrenches = wrench;
  srv_client.call(srv);

  std::cout << "service has been called" << std::endl;
}