#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "tf2_ros/transform_listener.h"
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <ur_dashboard_msgs/AddToLog.h>
#include <std_srvs/Trigger.h>
#include <atomic>
#include <robot_listener/butterworth_filter.hpp>
#include <sensor_msgs/JointState.h>


std::vector<std::vector<double>> wrench;
std::vector<std::vector<double>> tcp;
std::vector<std::vector<double>> joint;
std::atomic<bool> gather_data(false);
std::atomic<bool> gather_wrench_data(false);
std::atomic<bool> gather_joint_data(false);

std::string filename = "RobotTrial";
std::string wrench_file_name = "wrenchTrial";
std::string joint_file_name = "jointTrial";


ButterworthFilter filter_;

std::vector<double> filter_a_;
std::vector<double> filter_b_;
std::deque<double> force_z_;

void chatterCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
if (gather_wrench_data == true)
{
	if (force_z_.size() > 1000)
	{
		force_z_.pop_front();
		force_z_.push_back(msg->wrench.force.z);
	}
	else 
	{
		force_z_.push_back(msg->wrench.force.z);
	}
	std::vector<double> z_values = filter_.filter(force_z_, filter_b_, filter_a_);
	std::vector<double> wrench_values;
	wrench_values.push_back(msg->header.stamp.sec);
	wrench_values.push_back(msg->header.stamp.nsec);
	wrench_values.push_back(msg->wrench.force.x);
	wrench_values.push_back(msg->wrench.force.y);
	wrench_values.push_back(msg->wrench.force.z);
	wrench_values.push_back(msg->wrench.torque.x);
	wrench_values.push_back(msg->wrench.torque.y);
	wrench_values.push_back(msg->wrench.torque.z);
	wrench.push_back(wrench_values);
}
}

void tcpCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
if (gather_data == true)
{
	if (msg->transforms.size() == 1)
	{
		std::vector<double> tcp_values;
		tcp_values.push_back(msg->transforms[0].header.stamp.sec);
		tcp_values.push_back(msg->transforms[0].header.stamp.nsec);
		tcp_values.push_back(msg->transforms[0].transform.translation.x);
		tcp_values.push_back(msg->transforms[0].transform.translation.y);
		tcp_values.push_back(msg->transforms[0].transform.translation.z);
		tcp_values.push_back(msg->transforms[0].transform.rotation.w);
		tcp_values.push_back(msg->transforms[0].transform.rotation.x);
		tcp_values.push_back(msg->transforms[0].transform.rotation.y);
		tcp_values.push_back(msg->transforms[0].transform.rotation.z);
		tcp.push_back(tcp_values);
	}
}
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if (gather_joint_data == true)
	{
		std::vector<double> joint_values;
		for (unsigned int i = 0; i < msg->position.size(); i++)
		{
			joint_values.push_back(msg->position[i]);
		}
		joint.push_back(joint_values);
	}
}

void write_to_file()
{
	std::ofstream myfile;


	myfile.open ("/home/marcus/project/project_in_advanced_robotics/trajectory_learning/tcp" + filename + ".csv"); //writes to home folder

	for (int i = 0; i < tcp.size(); i++)
	{
			myfile << tcp[i][0] << "," << tcp[i][1] << "," << tcp[i][2] << "," << tcp[i][3] << "," << tcp[i][4] << "," << tcp[i][5] << "," << tcp[i][6] << "," << tcp[i][7] << "," << tcp[i][8] << "\n";
	}


	myfile.close();

	ROS_INFO_STREAM("Done writing tcp data to file: tcp" << filename);
}

void write_wrench_to_file()
{
	std::ofstream myfile;
	myfile.open ("/home/marcus/project/project_in_advanced_robotics/wrench" + wrench_file_name + ".csv"); //writes to home folder


	for (int i = 0; i < wrench.size(); i++)
	{
		myfile << wrench[i][0] << "," << wrench[i][1] << "," << wrench[i][2] << "," << wrench[i][3] << "," << wrench[i][4] << "," << wrench[i][5] << "," << wrench[i][6] << "," << wrench[i][7] << "\n";
	}


	myfile.close();
	ROS_INFO_STREAM("Done writing wrench data to file: wrench" << filename);
}

void write_joint_to_file()
{
	std::ofstream myfile;
	myfile.open ("/home/marcus/project/project_in_advanced_robotics/trajectory_learning/demonstrations_joint/joint" + joint_file_name + ".csv"); //writes to home folder

	for (int i = 0; i < joint.size(); i++)
	{
		myfile << joint[i][0] << "," << joint[i][1] << "," << joint[i][2] << "," << joint[i][3] << "," << joint[i][4] << "," << joint[i][5] << "\n";
	}


	myfile.close();
	ROS_INFO_STREAM("Done writing joint data to file: joint" << joint_file_name);
}

bool startRecordingService(ur_dashboard_msgs::AddToLog::Request& req, ur_dashboard_msgs::AddToLog::Response& res)
{
	filename = req.message;
	gather_data = true;
	ROS_INFO_STREAM("Gathering data from /tf");

	res.answer = "succes";
	res.success = true;
	return true;
}

bool stopRecordingService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
	gather_data = false;
	write_to_file();
	tcp.clear();
	res.message = "succes";
	res.success = true;
	return true;	
}

bool startWrenchRecordingService(ur_dashboard_msgs::AddToLog::Request& req, ur_dashboard_msgs::AddToLog::Response& res)
{
	wrench_file_name = req.message;
	gather_wrench_data = true;
	ROS_INFO_STREAM("Gathering data from /wrench");

	res.answer = "succes";
	res.success = true;
	return true;
}

bool stopWrenchRecordingService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
	gather_wrench_data = false;
	write_wrench_to_file();
	wrench.clear();
	res.message = "succes";
	res.success = true;
	return true;	
}

bool startJointRecordingService(ur_dashboard_msgs::AddToLog::Request& req, ur_dashboard_msgs::AddToLog::Response& res)
{
	joint_file_name = req.message;
	gather_joint_data = true;
	ROS_INFO_STREAM("Gathering data from /joint_states");

	res.answer = "succes";
	res.success = true;
	return true;
}

bool stopJointRecordingService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
	gather_joint_data = false;
	write_joint_to_file();
	joint.clear();
	res.message = "succes";
	res.success = true;
	return true;	
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_listener", ros::init_options::NoSigintHandler);

	ros::NodeHandle n;

	double fps = 500, fc = 100;
	double FrequencyBands[2] = { 0.000015,fc/(fps/2) };
	int FiltOrd = 3;


	filter_a_ = filter_.ComputeDenCoeffs(FiltOrd, FrequencyBands[0], FrequencyBands[1]);
	filter_b_ = filter_.ComputeNumCoeffs(FiltOrd, FrequencyBands[0], FrequencyBands[1], filter_a_);

	ros::ServiceServer start_recording = n.advertiseService("robot_listener/start_recording", startRecordingService);
	ros::ServiceServer stop_recording = n.advertiseService("robot_listener/stop_recording", stopRecordingService);
	ros::ServiceServer start_wrench_recording = n.advertiseService("robot_listener/start_wrench_recording", startWrenchRecordingService);
	ros::ServiceServer stop_wrench_recording = n.advertiseService("robot_listener/stop_wrench_recording", stopWrenchRecordingService);
	ros::ServiceServer start_joint_recording = n.advertiseService("robot_listener/start_joint_recording", startJointRecordingService);
	ros::ServiceServer stop_joint_recording = n.advertiseService("robot_listener/stop_joint_recording", stopJointRecordingService);

	ros::Subscriber sub_tcp = n.subscribe("tf", 1000, tcpCallback);
	ros::Subscriber sub = n.subscribe("wrench", 1000, chatterCallback);
	ros::Subscriber sub_joint = n.subscribe("joint_states", 1000, jointCallback);

	ros::spin();

	return 0;
}



