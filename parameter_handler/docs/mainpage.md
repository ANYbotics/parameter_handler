# Parameter Handler Overview

`parameter_handler` provides runtime-tunable parameters for application code.

## Features

Core features:

- define default, minimum, and maximum values
- change parameters from anywhere in the application
- support Eigen matrices as parameters

Additional ROS features:

- set and get parameters through ROS services
- full matrix support with MultiArray messages
- GUI-based parameter tuning workflows

## Package Layout

- `parameter_handler`: common parameter abstractions
- `parameter_handler_std`: non-ROS implementation
- `parameter_handler_ros`: ROS integration
- `parameter_handler_msgs`: service and message definitions

## Dependencies

The core packages primarily depend on:

- `message_logger`
- `Eigen`

The ROS integration additionally requires `roscpp` and `std_msgs`.

## Building

Build the package in a catkin workspace with:

```bash
catkin build parameter_handler
```

## Further Documentation

- [Use the parameter handler in your program](page_use_para.md)
- [Extend the parameter handler with additional types](page_extend_para.md)
- [Legacy RQT GUI](../../parameter_handler_ros/docs/page_rqt_gui.md)
