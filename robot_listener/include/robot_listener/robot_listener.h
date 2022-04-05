#include <ros/ros.h>

class RobotListener 
{
    private:
        ros::NodeHandle n_;
        ros::Subscriber sub_;

    public:
        RobotListener();

        void chatterCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)

};