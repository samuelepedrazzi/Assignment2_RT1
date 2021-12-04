#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "Assignment2_RT1/Velocity_service.h"
#include "Assignment2_RT1/Velocity_message.h"

ros::ServiceClient client;
ros::Publisher pub;


char GetInput()
{
    char input;
    std::cout << "Please enter a command.\n" << std::endl;
    std::cin >> input;
    return input;
}
void UICallbackFunction(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    




}













int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system
    system("clear");
    ros::init(argc, argv, "robot_ui");
    ros::NodeHandle nh;

    // Define the subscriber
    ros::ServiceClient client = nh.serviceClient<Assignment2_RT1::Velocity_service>("/service");
    //pub = nh.advertise<Assignment2_RT1::Vel>("/vel",1);
	ros::Subscriber sub = nh.subscribe("/base_scan", 100, UICallbackFunction);
	
	return 0;
}