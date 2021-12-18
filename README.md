# Assignment 2 - [Research_Track_1](https://unige.it/en/off.f/2021/ins/51201.html?codcla=10635) , [Robotics Engineering](https://courses.unige.it/10635).
Robot simulator using ROS
================================

-----------------------

Introduction <img src="https://cdn-icons.flaticon.com/png/128/938/premium/938446.png?token=exp=1639580920~hmac=6e5542379ff653021e57312c3f419fde" width=40>
------------
 
The goal of the project is to create a robot that can 'drive autonomously' inside the [Autodromo Nazionale di Monza](https://www.monzanet.it/) while paying attention not to crash with the circuit limitations.

In 'Assignment2_RT1' can be found several folders:
* 'world': folder containing information about the characteristics of the simulator's world
* 'CMakeLists.txt': text file describing how to build the code and where to install it
* 'package.xml': XML file defining properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other catkin packages

* 'src': folder containing three C++ files ('controller.cpp','server.cpp' and 'user_interface.cpp') that implement, respectively, three nodes: one that controls the robot movement along the circuit, one that corresponds to the actual server which receive the client request from the user_interface node and the last that interacts with the user.

* 'srv': folder containing a custom ROS service ('ChangeVel.srv') with the goal of bringing the two previously stated nodes together.

* 'gitignore': file that specifies intentionally untracked files that Git should ignore, not showing because not relevant and necessary for the project.

Installing and running <img src="https://media4.giphy.com/media/I8PIclm22mhMfJq0qx/200w.webp?cid=790b7611805i1n117mn1y069gy09vka0j0sq3gaamfdro6ln&rid=200w.webp&ct=s" width=80>
-----------------------

The simulator requires [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__), which is a collection of software libraries and tools that assist in the development of robot applications, especially it runs flawlessly on the [Noetic Release of ROS](http://wiki.ros.org/noetic/Installation).
In order to run the simulation, first you have to run ROS (using ```$ roscore &``` and ```$ catkin_make``` ), then you should run this commands, one per console page:

```console
$ rosrun stage_ros stageros $(rospack find RT1_Assignment2)/world/my_world.world
```

(This specific command will open the circuit)


```console
$ rosrun Assingment2_RT1 controller
```
(This command executes the controller node that is used for autonomous driving and the service that is utilized to enhance or reduce the speed )

```console
$ rosrun Assingment2_RT1 user_interface
```
(This specific command will run the user interface node with which the speed can be managed)

Circuit environment
---------

Here's the Monza track used in this game:

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/world/tracciato.png)

ROS generate the track arena based on this image, using the file `.world` contined inside the `world` folder. 

Stageros
------------

By using libstage, the stageros node covers the Stage 2-D multi-robot simulator.
Stage is a program that replicates a world defined in a.world file.
This file contains information about the world, including barriers (which are typically represented as a bitmap and utilized as a kind of background), robots, and other items.

The node only exposes a subset of Stage's functionality via ROS.
It looks for Stage models of the types laser, camera, and location, and maps them to the ROS subjects listed below.
Stageros exits if at least one laser/camera and position model are not discovered. 
The subscription to the topic 'cmd_vel' from the 'geometry_msgs' package, which provides a ['Twist'](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) type message to express the velocity of a robot in free space, broken into its linear and angular parts, is a very useful feature of the Stageros Node.

The Stageros Node additionally uses the'sensor_msgs' package's 'base scan' topic, which provides a ['LaserScan'](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) type message to represent a single scan from a planar laser range-finder.


Aside from that, I utilized a regular service from the'std srvs' package called 'reset_positions.'

The'std srvs' package offers a sort of service called ['Empty'](https://docs.ros.org/en/api/std_srvs/html/srv/Empty.html), which exchanges no actual data with the client but has proven to be highly beneficial for resetting the robot's location to its beginning point. 

Implementation choices
--------------

Initially, there was implemented the code that allowed the robot to move autonomously within the environment. This allows publishers and subscribers to change their behavior with feedback from the robot.

Afterwards it has implemented a user interface for taking input from the keyboard and modifying the speed of the robot in the circuit. Thanks to the service that establishes communication between all nodes, all the changes can be calculated.

<p align="center">
    
<img src="https://github.com/samuelepedrazzi/Assignment2_RT1/blob/main/images/Ros_nodes.drawio.png" width="600" height="350">
    
</p>

In a few words, the user will provide an input to the user interface node that will either be valid ('+' ,'-', 'R/r','q') or a bad one, resulting in an error input on the console. This input will be processed by the service, which will release a float value that will be the increased speed value. The user interface will manage it and sends the updates to the controller node, which will read the real increasing value. The controller node will then communicate updated velocity information to the stageros node. Another responsibility assigned to the controller is to call service from the ROS library that resets the robot's position.

Controller node  <img src="https://media4.giphy.com/media/AQ9ITNdrDb6XhZxDtd/200w.webp?cid=790b7611ycpbu1vkn0w4lha1xn131bjf2x8r6uj2bckcsqkk&rid=200w.webp&ct=s" width=50>
--------------

The controller node has the ability to drive around the track endlessly, detecting straights and turns autonomously.
When the robot is approaching a turn, the node tells him to slow down so he can make the necessary adjustments.

In the primary purpose, I define a subscriber to the "/base_scan" topic and a publisher to the "/cmd_vel" topic, using the public / subscribe method. The base_scan topic is a publisher that can provide an array named 'ranges' with 720 items that correspond to the distances from obstacles in a range of 0 to 180 degrees; cmd_vel, on the other hand, can change robot velocity. The main idea behind the code is that whenever the array 'ranges' changes, the ControlRobotTrack() is called, and the linear velocity is changed based on the distances detected by the laser, thanks to the publishing on the cmd_vel topic.

Thanks to the function CheckDistance(), the robot can detect the shortest distance to the walls on its right, left and front.
In the function that manages the robot movement, there are initialized three arrays which helps me in identifying the distances in the different directions based on the base_scan topic:

```cpp
   right = checkDistance(range_view, 0, 100);
   
   left = checkDistance(range_view, 620, 720);
  
   front = checkDistance(range_view, 310, 410);
```

The function implemented looks for the minimum value in each of the three sets and decides what action to take:

* if the front wall is closer than 'front min = 1.5' meters, he checks the lateral distances: 
    * if the left distance is greater than the right distance, he gradually turns to the right 
    * alternatively, he slightly turns to the left.


Otherwise the robot travels straight, if the wall is further than the threshold, and the '/Velocity_service' value is used as the updated speed value.
The UI node manages '/Velocity_service' based on the information he receives from the '/service'.


The controller node then publishes the data to the '/cmd_vel' topic, which is used to control the robot's movement. 

## Flowchart

Here below can be found the main idea behind the controller node's way of implementation.

![alt text](https://github.com/samuelepedrazzi/Assignment2_RT1/blob/main/images/Controller_Node.drawio.png)


### Speed service


The service, runned by the controller node, controls the robot's speed and collaborates closely with the user interface node, which is responsible for interacting with the final user.
It just verifies the character received by the UI node and adjusts the speed accordingly.
When the button R is pressed, the server uses the '/reset positions' service to automatically reset the robot to its initial position and velocity. 

More specifically the server will accept the user interface node's client request.

The different client requests are handled using a switch-case statement. The '+' allows for acceleration, the '-' for deceleration, 'q' to shutdown the UI node and publishers and subscribers to this node, and the 'R/r' for calling the 'reset_position' function from the 'std_srvs' package: this utility made resetting the robot to its initial position relatively simple.

Important to notice is the ros shutdown() function which is used for killing all open subscriptions, publications, service calls, and service servers. By default roscpp also installs a SIGINT handler which will detect Ctrl-C and automatically shutdown for the user. It can be useful if we want to stop the server management and the controller communication with the user_interface immediately. 
Then if we want to reload and restart the robot just re-run the commands in the terminal to start the closed nodes.


User_interface node  <img src="https://media0.giphy.com/media/p90XvKCcFnKZHEta4y/200w.webp?cid=790b7611805i1n117mn1y069gy09vka0j0sq3gaamfdro6ln&rid=200w.webp&ct=s" width=150>
---------------
The user interface node, as its name implies, acts as a connection to the other parts of the project, communicating with other nodes, server, and controller.
It takes the terminal's input and sends a request to the server, which returns a response to the user_interface node.
This occurs entirely within the UICallbackFunction(). 

When an external input is received, it is relayed to the controller node, which replies by giving back the robot's acceleration degree.
A custom service called Velocity_service.srv is created to implement this client-server communication architecture.
The service's structure is as follows: 

``` xml
     char input
     ---
     float32 value
```

* char input is the character typed on the keyboard by the user: '+','-','R/r','q' are the valid inputs.

* The degree of robot acceleration delivered as a response from the server to the client is represented by a float32 number.

The former screen of the UI is shown as follows:

![alt text](https://github.com/samuelepedrazzi/Assignment2_RT1/blob/main/images/user_interface_showing.png)

Conclusions and possible future improvements
--------------

Overall, I was pleased with the job, especially since this was the first real project I'd attempted with ROS. I realized the tool's potential and what can be readily built with it.

For example, the robot might be improved by adding the ability to follow the wall in order to avoid zigzagging in specific situations. Furthermore, though it is not needed in the assignment, the best ratio between linear and angular velocity throughout the corner could be used to avoid collisions when turning.

Except for those enhancements, the robot has good behavior; the velocity cannot be increased too much, it can drive all the way around the circuit without problems and for many laps: for instance, with a speed of 7/8, the robot can drive all the way around the circuit without problems; with more velocity it may can sometimes collide the walls.

<img src= "https://media3.giphy.com/media/y6PJrkD2AiME0B9sin/200w.webp?cid=790b7611ldy5v2egge0z6e7a5qtx6i6npclvmsf4paamg4l1&rid=200w.webp&ct=s" width=100 height=60>

It might also be possible to modify the robot's speed dynamically, like real cars do: The robot can travel at a rate that is inversely proportional to the amount of straight time it has left. However, in this example, the user input will be rendered worthless.

