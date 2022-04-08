#include <gtest/gtest.h>
#include <ros/ros.h>

using ::testing::InitGoogleTest;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_parameter_handler_ros");
  ros::start();  // To make use of ROS time in output macros.

  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
