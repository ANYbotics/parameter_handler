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
 * ParameterHandlerRos.hpp
 *
 *  Created on: Sep 24, 2015
 *      Author: Christian Gehring
 */

#pragma once

#include <ros/ros.h>
#include <parameter_handler_msgs/SetIntegralParameter.h>
#include <parameter_handler_msgs/GetIntegralParameter.h>
#include <parameter_handler_msgs/SetFloatingPointParameter.h>
#include <parameter_handler_msgs/GetFloatingPointParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>

#include <parameter_handler_std/ParameterHandlerStd.hpp>
#include <mutex>

namespace parameter_handler_ros {

class ParameterHandlerRos : public parameter_handler_std::ParameterHandlerStd
{
 public:
  ParameterHandlerRos();
  virtual ~ParameterHandlerRos();
  void initializeServices();
  void shutdown();
  void setNodeHandle(ros::NodeHandle& nodeHandle);
  bool getParameterList(parameter_handler_msgs::GetParameterList::Request &req,
                        parameter_handler_msgs::GetParameterList::Response &res);

  bool setIntegralParameter(parameter_handler_msgs::SetIntegralParameterRequest &req,
                            parameter_handler_msgs::SetIntegralParameterResponse &res);

  bool getIntegralParameter(parameter_handler_msgs::GetIntegralParameterRequest &req,
                            parameter_handler_msgs::GetIntegralParameterResponse &res);

  bool setFloatingPointParameter(parameter_handler_msgs::SetFloatingPointParameterRequest &req,
                                 parameter_handler_msgs::SetFloatingPointParameterResponse &res);

  bool getFloatingPointParameter(parameter_handler_msgs::GetFloatingPointParameterRequest &req,
                                 parameter_handler_msgs::GetFloatingPointParameterResponse &res);


  bool isIntegralType(const parameter_handler::ParameterInterface & param) {
    return (  param.getType() == typeid(bool) ||
              param.getType() == typeid(char) ||
              param.getType() == typeid(char16_t) ||
              param.getType() == typeid(char32_t) ||
              param.getType() == typeid(wchar_t) ||
              param.getType() == typeid(signed char) ||
              param.getType() == typeid(short int) ||
              param.getType() == typeid(int) ||
              param.getType() == typeid(long int) ||
              param.getType() == typeid(long long int) ||
              param.getType() == typeid(unsigned char) ||
              param.getType() == typeid(unsigned short int) ||
              param.getType() == typeid(Eigen::Vector3i) );
  }

  bool isFloatingPointType(const parameter_handler::ParameterInterface & param) {
    return (  param.getType() == typeid(float) ||
              param.getType() == typeid(double) ||
              param.getType() == typeid(Eigen::Vector3d) );
  }

  template <typename ScalarType_, typename MultiArrayMsg_>
  void readScalarParamFromMessage(parameter_handler::ParameterInterface & param, const MultiArrayMsg_ & msg) {
    if( (msg.layout.dim.size() == 1 && msg.layout.dim[0].size == 1) ||
        (msg.layout.dim.size() == 2 && msg.layout.dim[0].size == 1 && msg.layout.dim[1].size == 1 ) ) {
      param.setValue(static_cast<ScalarType_>(msg.data[0]));
    }
    else {
      ROS_ERROR_STREAM("Wrong dimension for scalar parameter " << param.getName());
    }
  }

  template <typename MatrixType_, typename MultiArrayMsg_>
  void readMatrixParamFromMessage(parameter_handler::ParameterInterface & param, const MultiArrayMsg_ & msg) {
    if(msg.layout.dim.size() == 1 || msg.layout.dim.size() == 2) {
      int rows = msg.layout.dim[0].size;
      int cols = (msg.layout.dim.size() == 2) ? msg.layout.dim[1].size : 1;
      if(param.getValue<MatrixType_>().rows() == rows  && param.getValue<MatrixType_>().cols() == cols )
      {
        MatrixType_ m;
        for( int r = 0; r < rows; ++r ) {
          for( int c = 0; c < cols; ++c ) {
            m(r,c) = static_cast<typename MatrixType_::Scalar>(msg.data[r*cols + c]);
          }
        }
        param.setValue(m);
      }
      else {
        ROS_ERROR_STREAM("Wrong matrix size for parameter " << param.getName());
      }
    }
    else {
      ROS_ERROR_STREAM("Wrong dimension for matrix parameter " << param.getName());
    }
  }

  template <typename ScalarType_, typename MultiArrayMsg_>
  void writeScalarToMessage(const ScalarType_ & scalar, MultiArrayMsg_ & msg) {
    // Add scalar data dimension
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "data";
    msg.layout.dim[0].size = 1;
    msg.layout.dim[0].stride = 1;
    msg.data.resize(1);
    msg.data[0] = static_cast<std::int64_t>(scalar);
  }

  template <typename MatrixType_, typename MultiArrayMsg_>
  void writeMatrixToMessage(const MatrixType_ & matrix, MultiArrayMsg_ & msg) {
    // Add matrix data dimension
    if(matrix.size() > 0) {
      msg.layout.dim.resize(2);
      msg.layout.dim[0].label = "rows";
      msg.layout.dim[0].size = matrix.rows();
      msg.layout.dim[0].stride = matrix.size();
      msg.layout.dim[1].label = "cols";
      msg.layout.dim[1].size = matrix.cols();
      msg.layout.dim[1].stride = matrix.cols();

      msg.data.resize(matrix.size());
      for( int r = 0; r < matrix.rows(); ++r ) {
        for( int c = 0; c < matrix.cols(); ++c ) {
          msg.data[r*matrix.cols() + c] = static_cast<std::int64_t>(matrix(r,c));
        }
      }
    }
    else {
      ROS_ERROR_STREAM("Matrix size is zero.");
    }
  }

  template <typename ScalarType_, typename GetParamResponse_>
  void writeScalarParamToMessage(const parameter_handler::ParameterInterface & param, GetParamResponse_ & msg) {
    writeScalarToMessage(param.getValue<ScalarType_>(), msg.value_current);
    writeScalarToMessage(param.getMinValue<ScalarType_>(), msg.value_min);
    writeScalarToMessage(param.getMaxValue<ScalarType_>(), msg.value_max);
    writeScalarToMessage(param.getDefaultValue<ScalarType_>(), msg.value_default);
  }

  template <typename MatrixType_, typename GetParamResponse_>
  void writeMatrixParamToMessage(const parameter_handler::ParameterInterface & param, GetParamResponse_ & msg) {
    writeMatrixToMessage(param.getValue<MatrixType_>(), msg.value_current);
    writeMatrixToMessage(param.getMinValue<MatrixType_>(), msg.value_min);
    writeMatrixToMessage(param.getMaxValue<MatrixType_>(), msg.value_max);
    writeMatrixToMessage(param.getDefaultValue<MatrixType_>(), msg.value_default);
  }

 protected:
  ros::NodeHandle nodeHandle_;
  ros::ServiceServer getParameterListService_;
  ros::ServiceServer getIntegralParameterService_;
  ros::ServiceServer setIntegralParameterService_;
  ros::ServiceServer getFloatingPointParameterService_;
  ros::ServiceServer setFloatingPointParameterService_;
};

} /* namespace parameter_handler */

