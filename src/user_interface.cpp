#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "Assignment2_RT1/Velocity_service.h"
#include "Assignment2_RT1/Velocity_message.h"

// Defining a ServiceClient object 'client' and a publisher
ros::ServiceClient client;
ros::Publisher pub;

// Function to read the char taken as input
char GetInput()
{
    char input;
    std::cout << "Please enter a command.\n"
              << std::endl;
    std::cin >> input;
    return input;
}

// Function that acts as user interface
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
    std::cout << "************************************************************\n"
              << std::endl;

    // Define a service object that sends a request to the server
    Assignment2_RT1::Velocity_service service;

    // Initialize a starting boolean valid input to false and an array with the valid inputs
    bool valid_input = false;
    char valid_inputs[] = {'+', '-', 'R', 'r', 'q'};

    // Read the input
    char user_input = GetInput();

    // Scrool the array to check if the input is among the valid ones
    for (int i = 0; i < 5; i++)
    {
        if (valid_inputs[i] == user_input)
        {
            std::cout << "\nValid input!\n"
                      << std::endl;
            valid_input = true;
        }
    }

    // If the input is valid check which it is and chooses what to do
    if (valid_input = true)
    {
        // for every input show a visible feedback it is pressed and than
        // wait for the existance of the server, send the input as a request of the server and call it
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
        // Define a message to send the response to the controller node, according to the message defined and to the service response
        Assignment2_RT1::Velocity_message message;
        message.velocity_msg = service.response.value;
        pub.publish(message);
        std::cout << "Speed updated, the new value is: " << service.response.value << "\n\n";
        valid_input = false;
    }
    else
        std::cout << "Not valid input! Please try again.\n"
                  << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system
    system("clear");
    ros::init(argc, argv, "robot_ui");
    ros::NodeHandle nh;

    // Call the service with the client
    client = nh.serviceClient<Assignment2_RT1::Velocity_service>("/service");
    // Define the subscriber to the ros topic (/base_scan) and advertise the topic of the message
    ros::Subscriber sub = nh.subscribe("/base_scan", 1, UICallbackFunction);
    pub = nh.advertise<Assignment2_RT1::Velocity_message>("/Velocity_message", 1);

    ros::spin();
    return 0;
}