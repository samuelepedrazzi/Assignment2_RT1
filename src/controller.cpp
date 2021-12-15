#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "Assignment2_RT1/Velocity_message.h"

// Initializing the publisher and the message of type geometry_msgs::Twist
ros::Publisher pub;
geometry_msgs::Twist robot_vel;

// Define global variables for the frontal threshold and the initial velocity
float front_min = 1.5;
float vel = 1;

// Function to calculate the minimum distance among array values
float checkDistance(float angle_range[], float min_value, float max_value)
{
    // set the max distance for the laser to not occure of errors
    float value = 50;

    // check every element of the array and
    for (int i = min_value; i < max_value; i++)
    {
        // if the value is less then the distance, update it
        if (angle_range[i] < value)
            value = angle_range[i];
    }
    return value;
}

// Function to control the robot movement and manage its speed
void ControlRobotTrack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // Initializing the array of the laser scans and the variables in which there will be the actual values of distance for the specific range
    float range_view[721];
    float right, left, front;

    // Filling the array for the CheckDistance() function
    for (int i = 0; i <= 721; i++)
        range_view[i] = msg->ranges[i];

    // Assigning the values that represent the front, left and right directions (ranges of view)
    right = checkDistance(range_view, 0, 100);
    left = checkDistance(range_view, 620, 720);
    front = checkDistance(range_view, 310, 410);

    // check if less than the threshold set before and update the speed in such a way that the robot cannot collide with circuit delimitations,
    // so it manages to turn in the right way to continue the path in presence of curves
    if (front < front_min)
    {
        if (right < left)
        {
            robot_vel.linear.x = 0.2;
            robot_vel.angular.z = 1;
        }
        else if (left < right)
        {
            robot_vel.linear.x = 0.2;
            robot_vel.angular.z = -1;
        }
    }
    //otherwise, if the path in front of the robot is clear, let's update the velocity, according to the server response
    else if (front > front_min)
    {
        robot_vel.linear.x = vel;
        robot_vel.angular.z = 0;
    }

    // show the actual value of the speed
    ROS_INFO("Actual speed: @[%.2f]", vel);
    pub.publish(robot_vel);
}

// Function that has to take the msg value of the velocity
void ManageSpeed(const Assignment2_RT1::Velocity_message::ConstPtr &speed)
{
    std::cout << "New value of speed is: " << speed->velocity_msg << "\n";
    vel = speed->velocity_msg;
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    // Define subscribers to the message (/Velocity_message) and to the ros topic (/base_scan)
    ros::Subscriber sub = nh.subscribe("/base_scan", 1, ControlRobotTrack);
    ros::Subscriber sub2 = nh.subscribe("/Velocity_message", 1, ManageSpeed);

    // advertise the topic /cmd_vel with the publisher
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::spin();
    return 0;
}