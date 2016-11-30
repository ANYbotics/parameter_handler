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

ParameterHandlerRos::ParameterHandlerRos()
{

}

ParameterHandlerRos::~ParameterHandlerRos()
{

}
void ParameterHandlerRos::initializeServices() {

  getParameterListService_ = nodeHandle_.advertiseService("get_parameter_list", &ParameterHandlerRos::getParameterList, this);
  getIntegralParameterService_ = nodeHandle_.advertiseService("get_integral_parameter", &ParameterHandlerRos::getIntegralParameter, this);
  getFloatingPointParameterService_ = nodeHandle_.advertiseService("get_floating_point_parameter", &ParameterHandlerRos::getFloatingPointParameter, this);
  setIntegralParameterService_ = nodeHandle_.advertiseService("set_integral_parameter", &ParameterHandlerRos::setIntegralParameter, this);
  setFloatingPointParameterService_ = nodeHandle_.advertiseService("set_floating_point_parameter", &ParameterHandlerRos::setFloatingPointParameter, this);

}

void ParameterHandlerRos::shutdown() {
  getParameterListService_.shutdown();
  getIntegralParameterService_.shutdown();
  getFloatingPointParameterService_.shutdown();
  setIntegralParameterService_.shutdown();
  setFloatingPointParameterService_.shutdown();
}

void ParameterHandlerRos::setNodeHandle(ros::NodeHandle& nodeHandle) {
  nodeHandle_ = nodeHandle;
}


bool ParameterHandlerRos::getParameterList(parameter_handler_msgs::GetParameterList::Request &req,
                                           parameter_handler_msgs::GetParameterList::Response &res) {


  for (auto& parameter : params_) {
    res.parameters.push_back(parameter.first);
  }
  return true;
}


bool ParameterHandlerRos::setIntegralParameter(parameter_handler_msgs::SetIntegralParameterRequest &req,
                                               parameter_handler_msgs::SetIntegralParameterResponse &res) {
  parameter_handler::ParameterInterface param;

  if (getParam(req.name, param)) {

    std::lock_guard<std::mutex> lock(mutexParams_);
    res.success = true;

    if( param.getType() == typeid(bool) ) { readScalarParamFromMessage<bool, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(char) ) { readScalarParamFromMessage<char, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(char16_t) ) { readScalarParamFromMessage<char16_t, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(char32_t) ) { readScalarParamFromMessage<char32_t, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(wchar_t) ) { readScalarParamFromMessage<wchar_t, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(signed char) ) { readScalarParamFromMessage<signed char, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(short int) ) { readScalarParamFromMessage<short int, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(int) ) { readScalarParamFromMessage<int, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(long int) ) { readScalarParamFromMessage<long int, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(long long int) ) { readScalarParamFromMessage<long long int, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(unsigned char) ) { readScalarParamFromMessage<unsigned char, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(unsigned short int) ) { readScalarParamFromMessage<unsigned short int, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(unsigned int) ) { readScalarParamFromMessage<unsigned int, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(unsigned long int) ) { readScalarParamFromMessage<unsigned long int, std_msgs::Int64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(Eigen::Vector3i) ) { readMatrixParamFromMessage<Eigen::Vector3i, std_msgs::Int64MultiArray>(param, req.value); }
    else {
      ROS_ERROR("Parameter type is not supported");
      res.success = false;
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
    res.success = true;

    if( param.getType() == typeid(bool) ) { writeScalarParamToMessage<bool, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(char) ) { writeScalarParamToMessage<char, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(char16_t) ) { writeScalarParamToMessage<char16_t, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(char32_t) ) { writeScalarParamToMessage<char32_t, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(wchar_t) ) { writeScalarParamToMessage<wchar_t, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(signed char) ) { writeScalarParamToMessage<signed char, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(short int) ) { writeScalarParamToMessage<short int, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(int) ) { writeScalarParamToMessage<int, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(long int) ) { writeScalarParamToMessage<long int, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(long long int) ) { writeScalarParamToMessage<long long int, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(unsigned char) ) { writeScalarParamToMessage<unsigned char, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(unsigned short int) ) { writeScalarParamToMessage<unsigned short int, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(unsigned int) ) { writeScalarParamToMessage<unsigned int, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(unsigned long int) ) { writeScalarParamToMessage<unsigned long int, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else if( param.getType() == typeid(Eigen::Vector3i) ) { writeMatrixParamToMessage<Eigen::Vector3i, parameter_handler_msgs::GetIntegralParameterResponse>(param, res); }
    else {
      ROS_ERROR("Parameter type is not supported");
      res.success = false;
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
    res.success = true;

    if( param.getType() == typeid(float) ) { readScalarParamFromMessage<float, std_msgs::Float64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(double) ) { readScalarParamFromMessage<double, std_msgs::Float64MultiArray>(param, req.value); }
    else if( param.getType() == typeid(Eigen::Vector3d) ) { readMatrixParamFromMessage<Eigen::Vector3d, std_msgs::Float64MultiArray>(param, req.value); }
    else {
      ROS_ERROR("Parameter type is not supported.");
      res.success = false;
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
    res.success = true;

    if( param.getType() == typeid(float) ) { writeScalarParamToMessage<float, parameter_handler_msgs::GetFloatingPointParameterResponse>(param, res); }
    else if( param.getType() == typeid(double) ) { writeScalarParamToMessage<double, parameter_handler_msgs::GetFloatingPointParameterResponse>(param, res); }
    else if( param.getType() == typeid(Eigen::Vector3d) ) { writeMatrixParamToMessage<Eigen::Vector3d, parameter_handler_msgs::GetFloatingPointParameterResponse>(param, res); }
    else {
      ROS_ERROR("Parameter type is not supported");
      res.success = false;
    }
  }
  else {
    ROS_ERROR("Parameter not found.");
    res.success = false;
  }

  return true;
}

} /* namespace parameter_handler */
