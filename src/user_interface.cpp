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
    std::cout << "Please enter a command.\n"
              << std::endl;
    std::cin >> input;
    return input;
}
void UICallbackFunction(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::cout << "************************************************************" << std::endl;
    std::cout << "Welcome to the user interface, you can choose from a range of inputs to control the robot\n"
              << std::endl;
    std::cout << "'+': increment the linear velocity\n"
              << std::endl;
    std::cout << "'-': decrement the linear velocity\n"
              << std::endl;
    std::cout << "'R/r': reset the robot in the inital position in the circuit\n"
              << std::endl;
    std::cout << "'q': quit the user interface node\n"
              << std::endl;
    std::cout << "************************************************************" << std::endl;

    Assignment2_RT1::Velocity_service service;
    bool valid_input = false;
    char valid_inputs[] = {'+', '-', 'R', 'r', 'q'};
    char user_input = GetInput();
    for (int i = 0; i < 5; i++)
    {
        if (valid_inputs[i] == user_input)
        {
            std::cout << "Valid input!\n"
                      << std::endl;
            valid_input = true;
        }
    }
    if (valid_input = true)
    {
        if (user_input == '+')
        {
            std::cout << "You have pressed \"+\"! The robot is accelerating.\n"
                      << std::endl;
            client.waitForExistence();
            service.request.input = '+';
            client.call(service);
        }
        else if (user_input == '-')
        {
            std::cout << "You have pressed \"-\"! The robot is decelerating.\n"
                      << std::endl;
            client.waitForExistence();
            service.request.input = '-';
            client.call(service);
        }
        else if (user_input == 'R' || user_input == 'r')
        {
            std::cout << "You have pressed \"R/r\"! The robot is reset to the initial position.\n"
                      << std::endl;
            client.waitForExistence();
            service.request.input = user_input;
            client.call(service);
        }
        else if (user_input == 'q')
        {
            std::cout << "You have pressed \"q\"! Quit the user interface.\n"
                      << std::endl;
            client.waitForExistence();
            service.request.input = 'q';
            client.call(service);
        }
        Assignment2_RT1::Velocity_message message;
        message.velocity_msg = service.response.value;
        pub.publish(message);
        std::cout << "Speed updated, the new value is: " << service.response.value << "\n\n";
        valid_input=false;
    }
    else
        std::cout << "Not valid input! Please try again.\n"
                  << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system
    system("clear");
    ros::init(argc, argv, "robot_ui");
    ros::NodeHandle nh;

    // Define the subscriber
    client = nh.serviceClient<Assignment2_RT1::Velocity_service>("/service");
    ros::Subscriber sub = nh.subscribe("/base_scan", 1, UICallbackFunction);
    pub = nh.advertise<Assignment2_RT1::Velocity_message>("/Velocity_message", 1);

    ros::spin();
    return 0;
}