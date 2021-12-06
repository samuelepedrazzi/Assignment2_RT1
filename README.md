Assignment 2 - Research Track 1 
================================

-----------------------

Introduction
------------

The goal of the project is to create a robot that can 'drive autonomously' inside the [Autodromo Nazionale di Monza](https://www.monzanet.it/) while paying attention not to crash with the circuit limitations.'

In 'Assignment2_RT1' can be found several folders:
* 'world': folder containing information about the characteristics of the simulator's world
* 'CMakeLists.txt': text file describing how to build the code and where to install it
* 'package.xml': XML file defining properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other catkin packages

* 'src': folder containing three C++ files ('controller.cpp','server.cpp' and 'user_interface.cpp') that implement, respectively, three nodes: one that controls the robot movement along the circuit, one that corresponds to the actual server which receive the client request from the user_interface node and the last that interacts with the user.

* 'srv': folder containing a custom ROS service ('ChangeVel.srv') with the goal of bringing the two previously stated nodes together.
* 'msg': ROS uses a simplified messages description language for describing the data values that ROS nodes publish. The message used in this assignment is 'Velocity_message' which describes the speed value. /Velocity_message is managed by the user_interface node, according to what he receives from the /service node.
* 'gitignore': file that specifies intentionally untracked files that Git should ignore, not showing because not relevant and necessary for the project.


Installing and running 
-----------------------

The simulator requires [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__), which is a collection of software libraries and tools that assist in the development of robot applications, especially it runs flawlessly on the [Noetic Release of ROS](http://wiki.ros.org/noetic/Installation).
In order to run the simulation, first you have to run ROS (using ```$ roscore &``` and ```$ catkin_make``` ), then you should run this commands, one per console page:

```console
$ rosrun stage_ros stageros $(rospack find RT1_Assignment2)/world/my_world.world
```

(This particular command will open the game circuit)


```console
$ rosrun RT1_Assignment2 controller
```
(This particular command will run the controller node used to drive autonomously)

```console
$ rosrun RT1_Assignment2 server
```
(This particular command will run the service used to increase or decrease speed)

```console
$ rosrun RT1_Assignment2 UI
```
(This particular command will run the UI that can control the velocity)

Game environment
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
The subscription to the topic 'cmd vel' from the 'geometry_msgs' package, which provides a ['Twist'](https://docs.ros.org/en/api/geometry msgs/html/msg/Twist.html) type message to express the velocity of a robot in free space, broken into its linear and angular parts, is a very useful feature of the Stageros Node.

The Stageros Node additionally uses the'sensor msgs' package's 'base scan' topic, which provides a ['LaserScan'](https://docs.ros.org/en/api/sensor msgs/html/msg/LaserScan.html) type message to represent a single scan from a planar laser range-finder.


Aside from that, I utilized a regular service from the'std srvs' package called 'reset_positions.'

The'std srvs' package offers a sort of service called ['Empty'](https://docs.ros.org/en/api/std srvs/html/srv/Empty.html), which exchanges no actual data with the client but has proven to be highly beneficial for resetting the robot's location to its beginning point. 

Controller node  <img src="https://media4.giphy.com/media/AQ9ITNdrDb6XhZxDtd/200w.webp?cid=790b7611ycpbu1vkn0w4lha1xn131bjf2x8r6uj2bckcsqkk&rid=200w.webp&ct=s" width=50>
--------------

The controller node has the ability to drive around the track endlessly, detecting straights and turns autonomously.
When the robot is approaching a turn, the node tells him to slow down so he can make the necessary adjustments.


After the '/base scan' publisher subscribes to it, the controller uses all of the sensors data acquired by him.
This topic is made up of 720 _ranges_, each of which contains all of the detected distances.
Each sensor has a 1/4-degree field of vision and can observe from -90 to 90 degrees.


The controller node enters the 'ControlRobotTrack' function after receiving a message from '/base scan,' which filters all ranges except those from: -100° to -60°,  -20° to 20°,  60° to 100°. 

The function then looks for the minimum value in each of the three sets and decides what action to take:


* if the front wall is closer than 'front min = 2' meters, he checks the lateral distances: 
    * if the left distance is greater than the right distance, he gradually turns to the right 
    * alternatively, he slightly turns to the left.


Otherwise the robot travels straight, if the wall is further than the threshold, and the '/Velocity message' value is used as the speed value.
The UI node manages '/Velocity_message' based on the information he receives from the '/service' node.


The controller node then publishes the data to the '/cmd vel' topic, which is used to control the robot's movement. 


Service node
--------------

The service node controls the robot's speed and collaborates closely with the UI node, which is responsible for interacting with the final user.
It just verifies the character received by the UI node and adjusts the speed accordingly.
When the button R is pressed, the service uses the '/reset positions' service to automatically reset the robot to its initial position and velocity. 


<img src= "https://media3.giphy.com/media/y6PJrkD2AiME0B9sin/200w.webp?cid=790b7611ldy5v2egge0z6e7a5qtx6i6npclvmsf4paamg4l1&rid=200w.webp&ct=s" width=100 height=50>

UI node <img src="https://media0.giphy.com/media/p90XvKCcFnKZHEta4y/200w.webp?cid=790b7611805i1n117mn1y069gy09vka0j0sq3gaamfdro6ln&rid=200w.webp&ct=s" width=100>
------
