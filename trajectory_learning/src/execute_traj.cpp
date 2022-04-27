// Ros related
#include <ros/ros.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Trigger.h>

// Implemented controller
#include <geometry_msgs/WrenchStamped.h>
#include <hybrid_force_pos_controller/targetTraj.h>

#include <fstream>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kinfam.hpp>
#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include <kdl/framevel.hpp>


int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "execute_traj");
  ros::NodeHandle nh;


  cartesian_control_msgs::CartesianTrajectory traj;
  std::ifstream file;
  std::ofstream out_file;
  double time = 0.01; 
  double step = 0.002;

  cartesian_control_msgs::CartesianTrajectory init_traj;
  bool flag = false;
  cartesian_control_msgs::CartesianTrajectoryPoint init_point;

  std::vector<geometry_msgs::WrenchStamped> wrench;
  geometry_msgs::WrenchStamped cur_wrench;
  cur_wrench.wrench.force.x = 0.0;
  cur_wrench.wrench.force.y = 0.0;
  cur_wrench.wrench.force.z = 8.0; // Seems to be working
  cur_wrench.wrench.torque.x = 0.0;
  cur_wrench.wrench.torque.y = 0.0;
  cur_wrench.wrench.torque.z = 0.0;
  file.open("/home/marcus/project/project_in_advanced_robotics/trajectory_learning/tcp.csv");
  if (file.is_open())
  {
    std::string line;
    while (std::getline(file, line, '\n'))
    {
      cartesian_control_msgs::CartesianTrajectoryPoint cur_point;
      std::stringstream ss(line);
      std::string data;
      std::vector<double> elems;
      while (getline(ss, data, ','))
      {
        elems.push_back(atof(data.c_str()));
      }
      KDL::JntArray joints(6);
      KDL::Frame pose;
      
      // The UR publishes joint angles in this order [elbow shoulder_lift shoulder_pan wrist1 2 3]
      joints(0) = elems[2];
      joints(1) = elems[1];
      joints(2) = elems[0];
      joints(3) = elems[3];
      joints(4) = elems[4];
      joints(5) = elems[5];

      fk_solver.JntToCart(joints, pose);
      ros_controllers_cartesian::CartesianState state;
      state.p.x() = pose.p.x();
      state.p.y() = pose.p.y();
      state.p.z() = pose.p.z();
      pose.M.GetQuaternion(state.q.x(), state.q.y(), state.q.z(), state.q.w());
      out_file << "2" << "," << "2" << "," << state.p.x() << "," << state.p.y() << "," << state.p.z() << ","
               << state.q.w() << "," << state.q.x() << "," << state.q.y() << "," << state.q.z() << "\n";
      /*ros::Duration dur(time);
      cur_point.time_from_start = dur;
        init_point.time_from_start = dur;
      cur_point.pose.position.x = elems[2];
      cur_point.pose.position.y = elems[3];
      cur_point.pose.position.z = elems[4];
      cur_point.pose.orientation.w = elems[5];
      cur_point.pose.orientation.x = elems[6];
      cur_point.pose.orientation.y = elems[7];
      cur_point.pose.orientation.z = elems[8];
      traj.points.push_back(cur_point);
      time = time + step;
      //wrench.push_back(cur_wrench);

      if(!flag)
      {
        init_point.pose.position.x = elems[2];
        init_point.pose.position.y = elems[3];
        init_point.pose.position.z = elems[4];
        init_point.pose.orientation.w = elems[5];
        init_point.pose.orientation.x = elems[6];
        init_point.pose.orientation.y = elems[7];
        init_point.pose.orientation.z = elems[8];
        flag = true;
      }
      init_traj.points.push_back(init_point);
    }

  }
  file.close();


  time = 0.01; 
  step = 0.002;

  file.open("/home/marcus/project/project_in_advanced_robotics/forcestop.txt");
  if (file.is_open())
  {
    std::string line;
    while (std::getline(file, line, '\n'))
    {
      ros::Duration dur(time);
      cur_wrench.wrench.force.z = atof(line.c_str());
      time = time + step;
      wrench.push_back(cur_wrench);
    }

  }
  file.close();
  out_file.close();

  /*ros::ServiceClient srv_client = nh.serviceClient<hybrid_force_pos_controller::targetTraj>("target_hybrid");

  hybrid_force_pos_controller::targetTraj srv;
  std_srvs::Trigger std_trigger;

  srv.request.traj = traj;
  srv.request.wrenches = wrench;
  srv_client.call(srv);*/
}