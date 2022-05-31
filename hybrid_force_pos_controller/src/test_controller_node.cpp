#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hybrid_force_pos_controller/targetTraj.h> 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_control_node");
  ros::NodeHandle nh;

  cartesian_control_msgs::CartesianTrajectoryPoint cur_point;
  cartesian_control_msgs::CartesianTrajectory traj;
  ros::Duration dur(3.0);
  cur_point.pose.position.x = 0.4;
  cur_point.pose.position.y = 0.4;
  cur_point.pose.position.z = 0.4;
  cur_point.pose.orientation.w = 1.0;
  cur_point.pose.orientation.x = 0.0;
  cur_point.pose.orientation.y = 0.0;
  cur_point.pose.orientation.z = 0.0;
  cur_point.time_from_start = dur;
  traj.points.push_back(cur_point);
  cur_point.pose.position.x = 0.5;
  cur_point.pose.position.y = 0.5;
  cur_point.pose.position.z = 0.5;
  dur += dur;
  cur_point.time_from_start = dur;
  traj.points.push_back(cur_point);

  std::vector<geometry_msgs::WrenchStamped> wrench;
  geometry_msgs::WrenchStamped cur_wrench;
  cur_wrench.wrench.force.x = 0.0;
  cur_wrench.wrench.force.y = 0.0;
  cur_wrench.wrench.force.z = 0.0; // Seems to be working
  cur_wrench.wrench.torque.x = 0.0;
  cur_wrench.wrench.torque.y = 0.0;
  cur_wrench.wrench.torque.z = 0.0;
  wrench.push_back(cur_wrench);
  wrench.push_back(cur_wrench);

  ros::ServiceClient srv_client = nh.serviceClient<hybrid_force_pos_controller::targetTraj>("target_hybrid");
  hybrid_force_pos_controller::targetTraj srv;
  srv.request.traj = traj;
  srv.request.wrenches = wrench;
  srv_client.call(srv);

  std::cout << "service has been called" << std::endl;

  return 0;
}