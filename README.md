# Université de Bourgogne

Robotics Engineering - ROS Project

## Report

**Authors**

Hong Win SOON, Theo PETITJEAN, Fabio RIBERO, Kumar ANKUSH

**Supervisors**

Ralph SEULIN, David FOFI, Raphael DUVERNE, Marc BLANCHON, Thibault CLAMENS


## Table of Contents
- [Introduction](#introduction)
  - [Overview](#overview)
- [Objectives](#objectives)


# Introduction

ROS or Robot Operating System is a meta-operating system used to operate robots or robotic hardware. A meta-operating system can be understood as an abstraction layer over an existing operating system and in that it provides us with a framework to test and design our robot.

It has a range of supported programming languages namely, C++ and Python and is a popular among the open source community. As such, there are a wide range of packages and codes that are available to us to choose from as well as extensive documentation on its operation. Utilising the freely available in-built and additional packages, ROS can give finer control of the robot without writing all control methods from scratch.


### Overview

The purpose of this project is to familiarize with ROS. Using unique packages pre-built by developers of ROS, we perform multiple actions of movement, localization, map building and then autonomous navigation. We will break down the process and discuss them in details further below.

# Objectives

The objectives in this project are enumerated below:

**1. Initialization and Navigation of a robot, Turtlebot 2 in this case**

For initialization and navigation of the robot, we used _turtlebot\_vibot_ package compiled by the Robotics Lab, France team at Condorcet (https://github.com/roboticslab-fr), University of Burgundy.


**2. Construction of a local map of the environment of the robot**

Using the same package we build the map of the robotics lab of Condorcet. 

**3. Movement of robot in the newly built map**

This step uses 'Rviz' to move the robot in the custom built map, this time within the confined boundaries without manual control.

**4. Defining the position points in the map for autonomous navigation**

Since the newly built map is a planar map, landmark position points in the map can be denoted by 2D vector.

**5. Using packages for reading QR code at the position points**

We used Python libraries for reading the QR codes at landmark points using the Kinect RGB camera.

**6. Play sound once QR code is detected**

For playing the sound upon successful registering of the QR code using another package called _sound-publisher_.

**7. Creating own launch files to unify all the functions**

Writing script for one launch file in order to launch all necessary scripts at once instead of calling them once at a time.

# Methodology

Let's break down certain important concepts of ROS and how it works.

- [_NODE_](http://wiki.ros.org/Nodes) : A node can be understood as a task, just like processes in an operating system. A node is a program to execute certain functions. By subscribing to a 'topic' and processing the information, a node then publishes the processed information to another 'topic'. 

- [_TOPIC_](http://wiki.ros.org/Topics) : A topic is a processed information index which has certain messages inside post- or pre-processing of information from a node. One can check existing topics by using command _rostopic_. 

- [_MESSAGES_](http://wiki.ros.org/Messages) : The information inside the various topics. Each topic, depending on the type of topic, has different information and may continuously publish information to the system.

- [_MASTER_](http://wiki.ros.org/Master) : It us the 'master' node. A system has unique master nodes for different devices. If an electronic system is interacting with different components which were not designed to work together natively, those components will have different master nodes which will publish processed information over a topic which can be subscribed by another master node.

- [_ROSLAUNCH_](http://wiki.ros.org/roslaunch) : It is the command to launch ROS commands which will in turn start the different nodes.

- [_RQT\_GRAPH_](http://wiki.ros.org/rqt_graph) : A visual representation of the nodes and topics of the system with connections showing where the nodes originate and which topics are being published and subscribed by which systems.

- [_ROSOUT_](http://wiki.ros.org/rosout) : Console log reporting system in ROS.


The packages we have used in our project are:
 - [*turtlebot\_vibot*](https://github.com/roboticslab-fr) - 
 Contains nodes which can initialize and operate the Kobuki base of the Turtlebot, the Kinect and the LIDAR connected to a computer. The package also has nodes for controlling the Kobuki base for navigation and publishing the odometry data with relevant topics in the subpackage _turtlebot\_vibot\_nav_. This subpackage also contains amcl (Adaptive Monte Carlo Localization) node to build a map with localization data which is fetched from LIDAR and Kinect. 

 - [*rbx1*](https://github.com/pirobot/rbx1) - 
 ROS By Example or rbx, contains multiple nodes which make use of various control nodes on both the master and the client terminal. We exclusively make use of a Python script in the subpackage _cv\_bridge\_demo.py_ in '_rbx1\_vision_' which we have modified to add our script to detect the QR code from Kinect's camera using OpenCV libraries.

 - [custom package]()

The master and client are assigned their own WiFi modules, connected over a local network and a given static IPs. Usually, the client cannot be connected to two network adaptors at the same time. Once the client and the master have been turned on and connected over the local network, we run execute the custom launch file which in turn calls for execution of other launch files from the _turtlebot\_vibot_ 
























