/**
 * @authors     Magnus Gaertner
 * @affiliation ANYbotics
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <parameter_handler_msgs/SetFloatingPointParameter.h>
#include <parameter_handler/parameter_handler.hpp>

#include "parameter_handler_ros/parameter_handler_ros.hpp"

namespace ph = parameter_handler;
namespace phr = parameter_handler_ros;

static bool setFloatingPointParameterUsingService(const std::string& name, double value) {
  ros::NodeHandle nh{"~"};
  ros::ServiceClient client{
      nh.serviceClient<parameter_handler_msgs::SetFloatingPointParameter>("parameter_handler_ros/set_floating_point_parameter")};

  parameter_handler_msgs::SetFloatingPointParameter srv;
  srv.request.name = name;
  parameter_handler_ros::writeScalarToMessage(value, srv.request.value);

  client.call(srv);
  return static_cast<bool>(srv.response.success);
}

struct HandledParameters {
  ph::Parameter<double> param1_{"param1", 0.0, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()};
  ph::Parameter<int> param2_{"param2", 0, -10, 10};
  ph::Parameter<bool> param3_{"param3", false, false, true};
};

//#ifdef USE_BOOST_PFR
TEST(TestParameterHandlerRos, testRegisterParameterStruct) {  // NOLINT
  // Setup an asynchronous spinner to simplify the testing logic.
  ros::AsyncSpinner spinner{2};
  spinner.start();

  // Register a ros parameter handler
  ros::NodeHandle nh_{"~"};
  phr::setParameterHandlerRos(&nh_);

  // Instantiate a struct of parameters and register it.
  HandledParameters parameters;
#ifdef USE_BOOST_PFR
  std::cout << "testing with registerParameterStruct" << std::endl;
  ph::registerParameterStruct(parameters);
#else
  std::cout << "testing without registerParameterStruct" << std::endl;
  parameter_handler::handler->addParam(parameters.param1_);
  parameter_handler::handler->addParam(parameters.param2_);
  parameter_handler::handler->addParam(parameters.param3_);
#endif

  // Check that we can set the parameter from a ros service call.
  // This would normally be done from a different node.
  EXPECT_TRUE(setFloatingPointParameterUsingService("param1", 42.0));
  EXPECT_FLOAT_EQ(parameters.param1_.getValue(), 42.0);

// Remove parameter handler.
#ifdef USE_BOOST_PFR
  ph::removeParametersStruct(parameters);
#else
  parameter_handler::handler->removeParam(parameters.param1_.getName());
  parameter_handler::handler->removeParam(parameters.param2_.getName());
  parameter_handler::handler->removeParam(parameters.param3_.getName());
#endif

  ph::handler->cleanup();
  spinner.stop();
}
