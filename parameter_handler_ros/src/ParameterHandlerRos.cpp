/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, C. Dario Bellicoso, Christian Gehring
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * ParameterHandlerRos.cpp
 *
 *  Created on: Sep 24, 2015
 *      Author: gech
 */

#include "parameter_handler_ros/ParameterHandlerRos.hpp"
#include <parameter_handler/Parameter.hpp>

namespace parameter_handler_ros {

using namespace parameter_handler;

ParameterHandlerRos::ParameterHandlerRos():
    nodeHandle_(nullptr)
{

}

ParameterHandlerRos::~ParameterHandlerRos()
{
  this->shutdown();
}

void ParameterHandlerRos::initializeServices() {

  if(nodeHandle_) {
    getParameterListService_ = nodeHandle_->advertiseService("parameter_handler_ros/get_parameter_list", &ParameterHandlerRos::getParameterList, this);
    getIntegralParameterService_ = nodeHandle_->advertiseService("parameter_handler_ros/get_integral_parameter", &ParameterHandlerRos::getIntegralParameter, this);
    getFloatingPointParameterService_ = nodeHandle_->advertiseService("parameter_handler_ros/get_floating_point_parameter", &ParameterHandlerRos::getFloatingPointParameter, this);
    setIntegralParameterService_ = nodeHandle_->advertiseService("parameter_handler_ros/set_integral_parameter", &ParameterHandlerRos::setIntegralParameter, this);
    setFloatingPointParameterService_ = nodeHandle_->advertiseService("parameter_handler_ros/set_floating_point_parameter", &ParameterHandlerRos::setFloatingPointParameter, this);
  }
  else {
    MELO_WARN("[ParameterHandlerRos] No nodehandle set.");
  }
}

void ParameterHandlerRos::shutdown() {
  getParameterListService_.shutdown();
  getIntegralParameterService_.shutdown();
  getFloatingPointParameterService_.shutdown();
  setIntegralParameterService_.shutdown();
  setFloatingPointParameterService_.shutdown();
}

void ParameterHandlerRos::setNodeHandle(ros::NodeHandle* nodeHandle) {
  nodeHandle_ = nodeHandle;
}

bool ParameterHandlerRos::cleanup() {
  parameter_handler_std::ParameterHandlerStd::cleanup();
  this->shutdown();
  nodeHandle_ = nullptr;
  return true;
}

bool ParameterHandlerRos::getParameterList(parameter_handler_msgs::GetParameterList::Request &req,
                                           parameter_handler_msgs::GetParameterList::Response &res) {
  for (auto& parameter : params_) {
    bool integralType = isType<PH_ROS_INTEGRAL_TYPES>(parameter.second);

    // Only integral and floating point types are supported
    if(integralType || isType<PH_ROS_FLOATING_POINT_TYPES>(parameter.second)) {
      res.parameters.push_back(parameter.first);
      res.isIntegral.push_back(integralType);
    }
  }
  return true;
}


bool ParameterHandlerRos::setIntegralParameter(parameter_handler_msgs::SetIntegralParameterRequest &req,
                                               parameter_handler_msgs::SetIntegralParameterResponse &res) {
  parameter_handler::ParameterInterface param;

  if (getParam(req.name, param)) {

    std::lock_guard<std::mutex> lock(mutexParams_);
    res.success = readScalarParamFromMessage<std_msgs::Int64MultiArray, PH_ROS_INTEGRAL_SCALAR_TYPES>(param, req.value) ||
                  readMatrixParamFromMessage<std_msgs::Int64MultiArray, PH_ROS_INTEGRAL_MATRIX_TYPES>(param, req.value);

    if(!res.success) {
      ROS_ERROR("Reading parameter from msg failed.");
    }
  }
  else {
    ROS_ERROR("Parameter not found.");
    res.success = false;
  }

  return true;
}

bool ParameterHandlerRos::getIntegralParameter(parameter_handler_msgs::GetIntegralParameterRequest &req,
                                               parameter_handler_msgs::GetIntegralParameterResponse &res) {
  parameter_handler::ParameterInterface param;

  if(getParam(req.name, param)) {

    std::lock_guard<std::mutex> lock(mutexParams_);
    res.success = writeScalarParamToMessage<parameter_handler_msgs::GetIntegralParameterResponse, PH_ROS_INTEGRAL_SCALAR_TYPES>(param, res) ||
                  writeMatrixParamToMessage<parameter_handler_msgs::GetIntegralParameterResponse, PH_ROS_INTEGRAL_MATRIX_TYPES>(param, res);

    if(!res.success) {
      ROS_ERROR("Writing integral parameter to msg failed.");
    }

  }
  else {
    ROS_ERROR("Parameter not found.");
    res.success = false;
  }

  return true;
}

bool ParameterHandlerRos::setFloatingPointParameter(parameter_handler_msgs::SetFloatingPointParameterRequest &req,
                                                    parameter_handler_msgs::SetFloatingPointParameterResponse &res) {
  parameter_handler::ParameterInterface param;

  if (getParam(req.name, param)) {

    std::lock_guard<std::mutex> lock(mutexParams_);
    res.success = readScalarParamFromMessage<std_msgs::Float64MultiArray, PH_ROS_FLOATING_POINT_SCALAR_TYPES>(param, req.value) ||
                  readMatrixParamFromMessage<std_msgs::Float64MultiArray, PH_ROS_FLOATING_POINT_MATRIX_TYPES>(param, req.value);

    if(!res.success) {
      ROS_ERROR("Reading parameter from msg failed.");
    }

  }
  else {
    ROS_ERROR("Parameter not found.");
    res.success = false;
  }

  return true;
}

bool ParameterHandlerRos::getFloatingPointParameter(parameter_handler_msgs::GetFloatingPointParameterRequest &req,
                                                    parameter_handler_msgs::GetFloatingPointParameterResponse &res) {
  parameter_handler::ParameterInterface param;

  if(getParam(req.name, param)) {

    std::lock_guard<std::mutex> lock(mutexParams_);
    res.success = writeScalarParamToMessage<parameter_handler_msgs::GetFloatingPointParameterResponse, PH_ROS_FLOATING_POINT_SCALAR_TYPES>(param, res) ||
                  writeMatrixParamToMessage<parameter_handler_msgs::GetFloatingPointParameterResponse, PH_ROS_FLOATING_POINT_MATRIX_TYPES>(param, res);

    if(!res.success) {
      ROS_ERROR("Writing floating point parameter to msg failed.");
    }

  }
  else {
    ROS_ERROR("Parameter not found.");
    res.success = false;
  }

  return true;
}

} /* namespace parameter_handler */
