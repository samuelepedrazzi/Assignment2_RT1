#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

//#define SIZE 100

ros::Publisher pub;
geometry_msgs::Twist robot_vel;

float front_min = 2;
float front_max = 2;

float checkDistance(float angle_range[], float min_value, float max_value)
{
	float value = 50;
    for (int i = min_value; i<max_value; i++)
    {
        if(angle_range[i]< value) value = angle_range[i];
    }
    return value;
}

void ControlRobotTrack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float range_view[720];

    float left, right, front; 

    for (int i=0; i<=720; i++) range_view[i] = msg->ranges[i];

    left = checkDistance(range_view, 0, 100);
    right = checkDistance(range_view, 620, 720);
    front = checkDistance(range_view, 310, 410);


    if(front<front_min)
    {
        if(left<right)
        {
            robot_vel.linear.x = 0.2;
            robot_vel.angular.y = 1;
        }
        else if(right<left)
        {
            robot_vel.linear.x = 0.2;
            robot_vel.angular.y = -1;
        }
    }
    else if(front>front_min)
    {
        robot_vel.linear.x = 1;
        robot_vel.angular.y = 0;
    }

    ROS_INFO("Right: @[%f]", 0);    
    pub.publish(robot_vel);
}


int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    // Define the subscriber to turtle's position
    ros::Subscriber sub = nh.subscribe("/base_scan", 1, ControlRobotTrack);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);   
    ros::spin();
    return 0;
}
