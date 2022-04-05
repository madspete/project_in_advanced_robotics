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


std::vector<std::vector<double>> wrench;
std::vector<std::vector<double>> tcp;

std::string filename = "RobotTrial";

void chatterCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
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

void tcpCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
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

void mySigintHandler(int sig)
{
    int sync = 0;
    bool flag = false;
    for(int i = 0;i < wrench.size();++i)
    {
        for (int j = 0; j < tcp.size(); j++)
        {
            if (wrench[i][0] == tcp[j][0] && wrench[i][1] == tcp[j][1])
            {
                sync = j;
                flag = true;
                break;
            }
            
        }
        if (flag)
        {
            break;
        }        
    }

    for (int i = 0; i < sync; ++i)
    {
        ROS_INFO_STREAM("Removing from tcp");
        tcp.erase(tcp.begin());
    }
    

    ROS_INFO_STREAM("sync is: " << sync);


    std::ofstream myfile;
    myfile.open ("wrench" + filename + ".csv"); //writes to home folder


    for (int i = 0; i < wrench.size(); i++)
    {
        myfile << wrench[i][0] << "," << wrench[i][1] << "," << wrench[i][2] << "," << wrench[i][3] << "," << wrench[i][4] << "," << wrench[i][5] << "," << wrench[i][6] << "," << wrench[i][7] << "\n";
    }
  
  
    myfile.close();

    ROS_INFO_STREAM("Done writing wrench data to file: wrench" << filename);


    myfile.open ("tcp" + filename + ".csv"); //writes to home folder

    for (int i = 0; i < tcp.size(); i++)
    {
        myfile << tcp[i][0] << "," << tcp[i][1] << "," << tcp[i][2] << "," << tcp[i][3] << "," << tcp[i][4] << "," << tcp[i][5] << "," << tcp[i][6] << "," << tcp[i][7] << "," << tcp[i][8] << "\n";
    }
  
  
    myfile.close();

    ROS_INFO_STREAM("Done writing tcp data to file: tcp" << filename);
  
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_listener", ros::init_options::NoSigintHandler);

    ros::NodeHandle n;

    if (argc == 2)
    {
        filename += argv[1];
    }


    ROS_INFO_STREAM("Gathering data from /wrench");
    ros::Subscriber sub = n.subscribe("wrench", 1000, chatterCallback);

    ROS_INFO_STREAM("Gathering data from /tf");
    ros::Subscriber sub_tcp = n.subscribe("tf", 1000, tcpCallback);


    signal(SIGINT, mySigintHandler);
    ros::spin();

    return 0;
}



