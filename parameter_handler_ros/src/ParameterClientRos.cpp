/*!
 * @file    ParameterClientRos.cpp
 * @author  Francisco Giraldez
 * @date    18.03.19
 */

// parameter_handler_msgs
#include <parameter_handler_msgs/GetFloatingPointParameter.h>
#include <parameter_handler_msgs/GetIntegralParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>
#include <parameter_handler_msgs/SetFloatingPointParameter.h>
#include <parameter_handler_msgs/SetIntegralParameter.h>

// parameter_handler_ros
#include <parameter_handler/helper_methods.hpp>
#include <parameter_handler_ros/ParameterClientRos.hpp>

namespace parameter_handler_ros {

// using namespace parameter_handler;

ParameterClientRos::ParameterClientRos() : nodeHandle_(nullptr), serverNamespace_("") {}

void ParameterClientRos::initializeClients() {
  if (!nodeHandle_) {
    MELO_ERROR("[ParameterClientRos] No nodehandle set. Could not initialize service clients!")
  }
  getParameterListClient_ = nodeHandle_->serviceClient<parameter_handler_msgs::GetParameterList>(
      "/" + serverNamespace_ + "/parameter_handler_ros/get_parameter_list");
  getIntegralParameterClient_ = nodeHandle_->serviceClient<parameter_handler_msgs::GetIntegralParameter>(
      "/" + serverNamespace_ + "parameter_handler_ros/get_integral_parameter");
  getFloatingPointParameterClient_ = nodeHandle_->serviceClient<parameter_handler_msgs::GetFloatingPointParameter>(
      "/" + serverNamespace_ + "parameter_handler_ros/get_floating_point_parameter");
  setIntegralParameterClient_ = nodeHandle_->serviceClient<parameter_handler_msgs::SetIntegralParameter>(
      "/" + serverNamespace_ + "parameter_handler_ros/set_integral_parameter");
  setFloatingPointParameterClient_ = nodeHandle_->serviceClient<parameter_handler_msgs::SetFloatingPointParameter>(
      "/" + serverNamespace_ + "parameter_handler_ros/set_floating_point_parameter");
  integralParameterChangeSubs_ = nodeHandle_->subscribe<parameter_handler_msgs::IntegralParameter>(
      "/" + serverNamespace_ + "parameter_handler_ros/notify_integral_parameter", 10, &ParameterClientRos::integralParameterChangeCallback,
      this);
  floatingPointParameterChangeSubs_ = nodeHandle_->subscribe<parameter_handler_msgs::FloatingPointParameter>(
      "/" + serverNamespace_ + "parameter_handler_ros/notify_floating_point_parameter", 10,
      &ParameterClientRos::floatingPointParameterChangeCallback, this);
}

void ParameterClientRos::shutdown() {
  getParameterListClient_.shutdown();
  getIntegralParameterClient_.shutdown();
  getFloatingPointParameterClient_.shutdown();
  setIntegralParameterClient_.shutdown();
  setFloatingPointParameterClient_.shutdown();
  integralParameterChangeSubs_.shutdown();
  floatingPointParameterChangeSubs_.shutdown();
}

void ParameterClientRos::setNodeHandle(ros::NodeHandle* nodeHandle) { nodeHandle_ = nodeHandle; }

void ParameterClientRos::setServerNamespace(const std::string& serverNamespace) { serverNamespace_ = serverNamespace; }

bool ParameterClientRos::hasParameter(const std::string& name) {
  parameter_handler_msgs::GetParameterList getParamListSrv;
  getParameterListClient_.call(getParamListSrv);
  bool paramFound = (std::find(getParamListSrv.response.parameters.begin(), getParamListSrv.response.parameters.end(), name) != getParamListSrv.response.parameters.end());
  return paramFound;
}

bool ParameterClientRos::cleanup() {
  this->shutdown();
  nodeHandle_ = nullptr;
  return true;
}

// template <typename ValueType_>
// bool setParameter(const std::string& name, const ValueType_& value) {}

// template <typename ValueType_>
// bool setParameter(const std::string& name, const ValueType_& value) {}

// bool sendIntegralParameter(const parameter_handler::RemoteParameterInterface& param) {
//   if (!parameter_handler::isType<parameter_handler::parameter_handler::PH_INTEGRAL_TYPES>(param)) {
//     MELO_ERROR_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of integral type!");
//     return false;
//   }
//   parameter_handler_msgs::SetIntegralParameterRequest req;
//   req.name = param.getName();
//   req.value = param.getValue();
//   setIntegralParameterClient_.call(req);
//   return true;
// }

// bool sendIntegralParameter(const parameter_handler::RemoteParameterInterface& param) {
//   if (!parameter_handler::isType<parameter_handler::parameter_handler::PH_FLOATING_POINT_TYPES>(param)) {
//     MELO_ERROR_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of floating point type!");
//     return false;
//   }
//   parameter_handler_msgs::SetIntegralParameterRequest req;
//   req.name = param.getName();
//   req.value = parame.getValue();
//   setFloatingPointParameterClient_.call(req);
//   return true;
// }

bool ParameterClientRos::getIntegralParameter(const std::string& name, parameter_handler::RemoteParameterInterface& param) {
  if (!parameter_handler::isType<parameter_handler::PH_FLOATING_INTEGRAL_TYPES>(param)) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of integral type.");
    return false;
  }
  parameter_handler_msgs::GetIntegralParameter getIntegralParamSrv;
  getIntegralParamSrv.request.name = name;
  getIntegralParameterClient_.call(getIntegralParamSrv);
  if (!getIntegralParamSrv.response.success) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of integral type in server " << serverNamespace_
                                                       << ".");
    return false;
  } else {
    param.setValue(getIntegralParamSrv.response.param.value_current);
    // TODO: solve other stuff?
    return true;
  }
}

bool ParameterClientRos::getIntegralParameter(parameter_handler::RemoteParameterInterface& param) {
  if (!parameter_handler::isType<parameter_handler::PH_INTEGRAL_TYPES>(param)) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of integral type.");
    return false;
  }
  parameter_handler_msgs::GetIntegralParameter getIntegralParamSrv;
  getIntegralParamSrv.request.name = param.getName();
  getIntegralParameterClient_.call(getIntegralParamSrv);
  if (!getIntegralParamSrv.response.success) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of integral type in server " << serverNamespace_
                                                       << ".");
    return false;
  } else {
    param.setValue(getIntegralParamSrv.response.param.value_current);
    // TODO: solve other stuff?
    return true;
  }
}

bool ParameterClientRos::getFloatingPointParameter(const std::string& name, parameter_handler::RemoteParameterInterface& param) {
  if (!parameter_handler::isType<parameter_handler::PH_FLOATING_POINT_TYPES>(param)) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of floating type.");
    return false;
  }
  parameter_handler_msgs::GetIntegralParameter getIntegralParamSrv;
  getIntegralParamSrv.request.name = name;
  getIntegralParameterClient_.call(getIntegralParamSrv);
  if (!getIntegralParamSrv.response.success) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of floating type in server " << serverNamespace_
                                                       << ".");
    return false;
  } else {
    param.setValue(getIntegralParamSrv.response.param.value_current);
    // TODO: solve other stuff?
    return true;
  }
}

bool ParameterClientRos::getFloatingPointParameter(parameter_handler::RemoteParameterInterface& param) {
  if (!parameter_handler::isType<parameter_handler::PH_FLOATING_POINT_TYPES>(param)) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of floating point type.");
    return false;
  }
  parameter_handler_msgs::GetIntegralParameter getIntegralParamSrv;
  getIntegralParamSrv.request.name = param.getName();
  getIntegralParameterClient_.call(getIntegralParamSrv);
  if (!getIntegralParamSrv.response.success) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " is not of floating point type in server " << serverNamespace_
                                                       << ".");
    return false;
  } else {
    param.setValue(getIntegralParamSrv.response.param.value_current);
    // TODO: solve other stuff?
    return true;
  }
}

// bool ParameterClientRos::getParameterList() {
//   for (auto& parameter : params_) {
//     bool integralType = parameter_handler::isType<parameter_handler::PH_INTEGRAL_TYPES>(parameter.second);

//     // Only integral and floating point types are supported
//     if (integralType || parameter_handler::isType<parameter_handler::PH_FLOATING_POINT_TYPES>(parameter.second)) {
//       res.parameters.push_back(parameter.first);
//       res.isIntegral.push_back(integralType);
//     }
//   }
//   return true;
// }

void ParameterClientRos::integralParameterChangeCallback(const parameter_handler_msgs::IntegralParameterConstPtr& msg) {
  for (auto& parameter : connectedParameters_) {
    if ((parameter.getName() == msg->name) && parameter_handler::isType<parameter_handler::PH_INTEGRAL_TYPES>(parameter)) {
      // parameter.setM

    } else {
      MELO_DEBUG_STREAM("[ParameterClientRos] Integral parameter " << msg->name
                                                                   << " not found in connected parameters. Ignoring notification.");
    }
  }
}

void ParameterClientRos::floatingPointParameterChangeCallback(const parameter_handler_msgs::FloatingPointParameterConstPtr& msg) {
  auto it = std::find_if(connectedParameters_.begin(), connectedParameters_.end(), [](parameter_handler::RemoteParameterInterface* param) {
    (param->getName() == msg->name) && parameter_handler::isType<parameter_handler::PH_FLOATING_POINT_TYPES>(*param)
  });
  if (it != connectedParameters_.end()) {
    connectedParameters_[it]->setValue(msg->value_current);
    // connectedParameters_[it].setValue(msg->value_current)
  } else {
    MELO_DEBUG_STREAM("[ParameterClientRos] Floating point parameter " << msg->name
                                                                       << " not found in connected parameters. Ignoring notification.");
  }
  return;
}

void ParameterClientRos::parameterChanged(const parameter_handler::RemoteParameterInterface& param) {
  if (parameter_handler::isType<parameter_handler::PH_INTEGRAL_TYPES>(param)) {
    //sendIntegralParameter(param);
  } else if (parameter_handler::isType<parameter_handler::PH_FLOATING_POINT_TYPES>(param)) {
    //sendFloatingPointParameter(param);
  } else {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " could not be updated due to unknown type!");
  }
  return;
}

bool ParameterClientRos::connectRemoteParameter(parameter_handler::RemoteParameterInterface& param) {
  if (!hasParameter(param.getName())) {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " could not be found in server " << serverNamespace_ << ".");
    return false;
  }
  if (parameter_handler::isType<parameter_handler::PH_INTEGRAL_TYPES>(param)) {
    getIntegralParameter(param);
  } else if (parameter_handler::isType<parameter_handler::PH_FLOATING_POINT_TYPES>(param)) {
    getFloatingPointParameter(param);
  } else {
    MELO_WARN_STREAM("[ParameterClientRos] Parameter " << param.getName() << " could not be connected due to unknown type!");
    return false;
  }
  param.addObserver(&this);
  connectedParameters_.push_back(&param);
  return true;
}

}  // namespace parameter_handler_ros
