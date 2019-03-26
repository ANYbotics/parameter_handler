/*!
 * @file    ParameterClientRos.hpp
 * @author  Francisco Giraldez
 * @date    18.03.19
 */

#pragma once

#include <parameter_handler_msgs/GetFloatingPointParameter.h>
#include <parameter_handler_msgs/GetIntegralParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>
#include <parameter_handler_msgs/SetFloatingPointParameter.h>
#include <parameter_handler_msgs/SetIntegralParameter.h>
#include <ros/ros.h>

#include <parameter_handler/ParameterObserverInterface.hpp>
#include <parameter_handler/RemoteParameterInterface.hpp>
#include <parameter_handler/type_macros.hpp>
#include <parameter_handler_ros/helper_methods.hpp>

#include <mutex>

namespace parameter_handler_ros {

class ParameterClientRos : public parameter_handler::ParameterObserverInterface {
 public:
  ParameterClientRos();
  ~ParameterClientRos() override = default;

  void initializeClients();
  void shutdown();

  void setNodeHandle(ros::NodeHandle* nodeHandle);

  void setServerNamespace(const std::string& serverNamespace);

  bool cleanup();

  bool hasParameter(const std::string& name);

  //bool getParameterList();
  // template<typename ValueType_>
  // bool setParameter(const std::string& name, const ValueType_& value);

  // template<typename ValueType_>
  // bool setParameter(const std::string& name, const ValueType_& value);

  bool sendIntegralParameter(const parameter_handler::RemoteParameterInterface& param);

  bool getIntegralParameter(const std::string& name, parameter_handler::RemoteParameterInterface& param);

  bool getIntegralParameter(parameter_handler::RemoteParameterInterface& param);

  bool sendFloatingPointParameter(const std::string& name);

  bool getFloatingPointParameter(const std::string& name, parameter_handler::RemoteParameterInterface& param);

  bool getFloatingPointParameter(parameter_handler::RemoteParameterInterface& param);
  
  void parameterChanged(const parameter_handler::RemoteParameterInterface& param) override;

  bool connectRemoteParameter(parameter_handler::RemoteParameterInterface& param);

 protected:
  ros::NodeHandle* nodeHandle_;
  std::string serverNamespace_;

  ros::ServiceClient getParameterListClient_;
  ros::ServiceClient getIntegralParameterClient_;
  ros::ServiceClient setIntegralParameterClient_;
  ros::ServiceClient getFloatingPointParameterClient_;
  ros::ServiceClient setFloatingPointParameterClient_;
  ros::Subscriber integralParameterChangeSubs_;
  ros::Subscriber floatingPointParameterChangeSubs_;

  void integralParameterChangeCallback(const parameter_handler_msgs::IntegralParameterConstPtr& msg);
  void floatingPointParameterChangeCallback(const parameter_handler_msgs::FloatingPointParameterConstPtr& msg);

  std::vector<parameter_handler::RemoteParameterInterface*> connectedParameters_; 

};

}  // namespace parameter_handler_ros
