#include "ros/ros.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

void enuCallback(const geometry_msgs::TransformStamped enu_msg)
{
    ned_msg.header = enu_msg.header;
    ned_msg.pose.position.x = 
    ned_msg.pose.position.y = 
    ned_msg.pose.position.z = 
    ned_msg.pose.orientation.x = 
    ned_msg.pose.orientation.y = 
    ned_msg.pose.orientation.z = 
    ned_msg.pose.orientation.w = 

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "enu2ned");

    ros::NodeHandle n;

    ros::Publisher ned_pub = n.advertise<geometry_msgs::PoseStamped.h>("end2ned",10);

    ros::Subscriber enu_sub = n.subscribe("<VICON_TOPIC_HERE>",10,enuCallback);

    geometry_msgs::PoseStamped ned_msg

    ros::spin();

    return 0;    
}