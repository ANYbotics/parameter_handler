# Parameter Handler

## Overview

These packages provide an interface for a parameter handler and some default parameter handlers.
A parameter handler is a tool to tune parameters defined in other classes.

[Documentation](http://docs.leggedrobotics.com/parameter_handler_doc/)

The software has been tested under ROS Indigo and Ubuntu 14.04.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author(s):** Christian Gehring, Gabriel Hottiger, C. Dario Bellicoso


## Building

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=parameter_handler)](http://rsl-ci.ethz.ch/job/parameter_handler/)

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

### Dependencies
* **[kindr](https://github.com/ethz-asl/kindr):** Kinematics and Dynamics for Robotics
* **[kindr_ros](https://github.com/ethz-asl/kindr_ros):** kindr ROS wrapper
* **[message_logger](https://bitbucket.org/ethz-asl-lr/message_logger):** Message logging utility
* **Eigen3:** Linear alegra library
* **gtest:** Google's unit testing framework

## Usage

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).

### Packages
* **parameter_handler:** Interface to parameter handler
* **parameter_handler_doc:** Doxygen documentation
* **parameter_handler_msgs:** ROS messages used by parameter_handler_ros
* **parameter_handler_ros:** An implementation of the parameter handler that uses ROS
* **parameter_handler_std:** An implementation of the parameter handler that is independent of ROS
* **rqt_parameters:** RQT plugin that provides a GUI for the parameter_handler_ros


