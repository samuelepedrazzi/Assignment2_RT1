#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "Assignment2_RT1/Velocity_service.h"
#include "std_srvs/Empty.h"

std_srvs::Empty reset;

float service_speed = 0;

bool UpdateVelocity(Assignment2_RT1::Velocity_service::Request &request, Assignment2_RT1::Velocity_service::Response &response)
{
    switch (request.input)
    {

    //increase speed
    case '+':
        service_speed += 1;
        request.input = 'q';
        break;

    //decrease speed
    case '-':
        if (service_speed >= 2)
        {
            service_speed -= 1;
            request.input = 'q';
        }
        break;

    //reset the robot in the initial position
    case 'r':
    case 'R':
        service_speed = 1;
        ros::service::call("/reset_positions", reset);
        break;
    case 'q':
        return false;
    default:
        break;
    }
    response.value = service_speed;
    ROS_INFO("Updated speed: @[%f]", response.value);   

    return true;
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system
    ros::init(argc, argv, "server");
    ros::NodeHandle nh;
    // Define the subscriber
    ros::ServiceServer service = nh.advertiseService("/service",UpdateVelocity);
    ros::spin();
    return 0;
}
