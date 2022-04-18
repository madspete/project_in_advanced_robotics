#include "trajectory_learning/gmm_gmr.hpp"

#include <ros/ros.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>

// Make a function to switch on the correct controllers - maybe this is needed

int main(int argc, char* argv[])
{

  // This must be called before anything else ROS-related
  ros::init(argc, argv, "gmm");

  // Align the demonstrated trajectories
  trajectory_learning::GMMAndGMR gmm_gmr("/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations");

  gmm_gmr.gmm_learn();
  std::vector<unsigned int > in = {7};
  std::vector<unsigned int > out = {0,1,2,3,4,5,6};
  std::vector<double> input_data;

  double step = 0.0001; //This can be adjusted to what is needed.
  for (double i = 0.0; i < 10; i = i +step)
  {
    input_data.push_back(i);
  }
  std::vector<Vector> target;
  gmm_gmr.gmr_calculation_fast_gmm(target, input_data, in, out);

  
  // First point in the trajectory cannot be 0.0 time, it should be a time different from zero.
  cartesian_control_msgs::FollowCartesianTrajectoryGoal trajectory_goal;
  cartesian_control_msgs::FollowCartesianTrajectoryGoal first_goal;
  double cur_time = 0.01;
  std::ofstream myfile;
  myfile.open ("/home/mads/git/project_in_advanced_robotics/trajectory_learning/test.csv");
  for (unsigned int i = 0; i < target.size(); ++i)
  {
    cartesian_control_msgs::CartesianTrajectoryPoint cur_point;
    ros::Duration dur(cur_time);
    cur_point.time_from_start = dur;
    cur_point.pose.position.x = target[i][0];
    cur_point.pose.position.y = target[i][1];
    cur_point.pose.position.z = target[i][2];
    cur_point.pose.orientation.w = target[i][3];
    cur_point.pose.orientation.x = target[i][4];
    cur_point.pose.orientation.y = target[i][5];
    cur_point.pose.orientation.z = target[i][6];
    
    trajectory_goal.trajectory.points.push_back(cur_point);
    cur_time = cur_time + step;
    if (i == 0)
    {
      cur_point.time_from_start = ros::Duration(10.0);
      first_goal.trajectory.points.push_back(cur_point);
    }
    myfile << cur_point.pose.position.x << "," << cur_point.pose.position.y << "," << cur_point.pose.position.z << "," << cur_point.pose.orientation.w  << "," << cur_point.pose.orientation.x << "," << cur_point.pose.orientation.y << "," << cur_point.pose.orientation.z << "\n";
  }

  myfile.close();

  gmm_gmr.write_data_to_file("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/means.txt", 
                             "/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/cov.txt",
                             "/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/priors.txt");

  // Move robot to initial trajectory point
  actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> publish_action("/pose_based_cartesian_traj_controller/follow_cartesian_trajectory", true);
  publish_action.waitForServer(ros::Duration(10));
  publish_action.sendGoal(first_goal);
  publish_action.waitForResult(ros::Duration(15));
  if (publish_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout << "Robot move succesfully to the first point executed succesfully " << std::endl;

  // Execute trajectory
  publish_action.sendGoal(trajectory_goal);
  publish_action.waitForResult(ros::Duration(15));
  if (publish_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout << "Trajectory executed succesfully" << std::endl;
}