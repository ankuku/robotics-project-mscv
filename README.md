# Université de Bourgogne

Robotics Engineering - ROS Project

## Report

**Authors**

Hong Win Soon

Theo Petitjean

Fabio Ribero

Kumar Ankush

**Supervisors**

Ralph SEULIN

David FOFI

Raphael DUVERNE

Marc BLANCHON

Thibault CLAMENS


## Table of Contents
- [Introduction](#introduction)
  - [Overview](#overview)


# Introduction

ROS or Robot Operating System is a meta-operating system used to operate robots or robotic hardware. A meta-operating system can be understood as an abstraction layer over an existing operating system and in that it provides us with a framework to test and design our robot.

It has a range of supported programming languages namely, C++ and Python and is a popular among the open source community. As such, there are a wide range of packages and codes that are available to us to choose from as well as extensive documentation on its operation. Utilising the freely available in-built and additional packages, ROS can give finer control of the robot without writing all control methods from scratch.


# Overview

The purpose of this project is to familiarize with ROS. Using unique packages pre-built by developers of ROS, we perform multiple actions of movement, localization, map building and then autonomous navigation. We will break down the process and discuss them in details further below.

The objectives in this project are enumerated below:

**1. Initialization and Navigation of a robot, Turtlebot 2 in this case**
For initialization and navigation of the robot, we used *turtlebot\_vibot* package compiled by the Robotics Lab, France team at Condorcet (https://github.com/roboticslab-fr), University of Burgundy.


**2. Construction of a local map of the environment of the robot**
Using the same package we build the map of the robotics lab of Condorcet. 

**3. Movement of robot in the newly built map**
This step uses 'Rviz' to move the robot in the custom built map, this time within the confined boundaries without manual control.

**4. Defining the position points in the map for autonomous navigation**
Since the newly built map is a planar map, landmark position points in the map can be denoted by 2D vector.

**5. Using packages for reading QR code at the position points**
We used Python libraries for reading the QR codes at landmark points using the Kinect RGB camera.

**6. Play sound once QR code is detected**
For playing the sound upon successful registering of the QR code using another package called *sound-publisher*.

**7. Creating own launch files to unify all the functions**

















