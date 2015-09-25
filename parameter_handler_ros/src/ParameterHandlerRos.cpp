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
  getParameterService_ = nodeHandle_.advertiseService("get_parameter", &ParameterHandlerRos::getParameter, this);
  setParameterService_ = nodeHandle_.advertiseService("set_parameter", &ParameterHandlerRos::setParameter, this);
}

void ParameterHandlerRos::shutdown() {
  getParameterListService_.shutdown();
  getParameterService_.shutdown();
  setParameterService_.shutdown();
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

bool ParameterHandlerRos::setParameter(parameter_handler_msgs::SetParameter::Request &req,
                                       parameter_handler_msgs::SetParameter::Response &res) {



  parameter_handler::ParameterInterface param;
  if (!getParam(req.name, param)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutexParams_);

  if (param.getType() == typeid(double)) {
    param.setValue(req.value);
  }
  else {
    ROS_ERROR("Parameter type is not supported");
  }
  return true;
}


bool ParameterHandlerRos::getParameter(parameter_handler_msgs::GetParameter::Request &req,
                                       parameter_handler_msgs::GetParameter::Response &res) {


  parameter_handler::ParameterInterface param;

  if(getParam(req.name, param)) {
    std::lock_guard<std::mutex> lock(mutexParams_);

    if (param.getType() == typeid(double)) {
      res.value_current = param.getValue<double>();
      res.value_min = param.getMinValue<double>();
      res.value_max = param.getMaxValue<double>();
      res.value_default = param.getDefaultValue<double>();
    }
    else {
      ROS_ERROR("Parameter type is not supported");
    }


    return true;
  }


  return false;
}

} /* namespace parameter_handler */
