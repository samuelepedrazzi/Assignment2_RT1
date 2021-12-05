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

* 'src': folder containing two C++ scripts ('robot controller.cpp' and 'robot GUI.cpp') that implement two nodes: one that controls the robot and does some operations on demand, and the other that interacts with the user and sends requests to the first.

* 'srv': folder containing a custom ROS service ('ChangeVel.srv') with the goal of bringing the two previously stated nodes together.
* 'gitignore': a file which duty is to avoid showing some files which are not necessary to the main target of the project.


Installing and running 
-----------------------

The simulator requires [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__) to be installed on the machine. In particular, the [Noetic Release of ROS](http://wiki.ros.org/noetic/Installation) was used.

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

Controller node
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

UI node
------
