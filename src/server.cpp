#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "Assignment2_RT1/Velocity_service.h"
#include "std_srvs/Empty.h"

// Defining the Empty object reset which is needed to reset the robot position and the first value of the speed
std_srvs::Empty reset;
float service_speed = 1;

// Function that returns true if an input has arrived and chooses what to do based on it:
// it is dedicated to the change of the velocity communicating with the user interface.
bool UpdateVelocity(Assignment2_RT1::Velocity_service::Request &request, Assignment2_RT1::Velocity_service::Response &response)
{
    switch (request.input)
    {

    //increase speed
    case '+':
        service_speed += 1;
        break;

    //decrease speed
    case '-':
        if (service_speed >= 2)
        {
            service_speed -= 1;
        }
        break;

    //reset the robot in the initial position by callig the service reset_positions
    case 'r':
    case 'R':
        service_speed = 1;
        ros::service::call("/reset_positions", reset);
        break;

    //if 'q' is the input received, the user interfacenode is closed
    case 'q':
        ros::shutdown();
        break;
    default:
        break;
    }
    // Updating and showing the changed value of speed
    response.value = service_speed;
    ROS_INFO("Updated speed: @[%.2f]", response.value);

    return true;
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system
    ros::init(argc, argv, "server");
    ros::NodeHandle nh;

     // Define the service and call the UpdateVelocity() function
    ros::ServiceServer service = nh.advertiseService("/service", UpdateVelocity);
    ros::spin();
    return 0;
}
