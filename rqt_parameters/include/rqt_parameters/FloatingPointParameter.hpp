/*
 * DoubleParameter.hpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

#pragma once

// rqt_parameters
#include "rqt_parameters/ParameterBase.hpp"

#include <ros/ros.h>

#include <parameter_handler_msgs/GetFloatingPointParameter.h>
#include <parameter_handler_msgs/SetFloatingPointParameter.h>

//! This class draws and handles a double parameter.
class FloatingPointParameter: public ParameterBase {
  Q_OBJECT
 public:
  FloatingPointParameter(const std::string& name,
                  QWidget* widget,
                  QGridLayout* grid,
                  ros::ServiceClient* getParameterClient,
                  ros::ServiceClient* setParameterClient,
                  size_t maxParamNameWidth):
                  ParameterBase(name, widget, grid, getParameterClient, setParameterClient, maxParamNameWidth)
  {

  }

  virtual ~FloatingPointParameter() {

  };

  template <typename GetParamResponse_>
  void readMatrixParamFromMessage(MatrixSpinBox * matrix, const GetParamResponse_ & msg) {
    if(msg.value_current.layout.dim.size() == 1 || msg.value_current.layout.dim.size() == 2) {
      int rows = msg.value_current.layout.dim[0].size;
      int cols = (msg.value_current.layout.dim.size() == 2) ? msg.value_current.layout.dim[1].size : 1;
      matrix->init(rows, cols);
      for( int r = 0; r < rows; ++r ) {
        for( int c = 0; c < cols; ++c ) {
          double val = msg.value_current.data[r*cols + c];
          double min = msg.value_min.data[r*cols + c];
          double max = msg.value_max.data[r*cols + c];
          matrix->getSpinbox(r,c)->setMinimum(min);
          matrix->getSpinbox(r,c)->setMaximum(max);
          matrix->getSpinbox(r,c)->setValue(val);
          matrix->getSpinbox(r,c)->setRange(min, max);
          matrix->getSpinbox(r,c)->setSingleStep(std::fabs( (max - min) / 10.0) );
          QString tooltip = QString("Min: ") + QString::number(min, 'f', 2) + QString(" / Max: ") + QString::number(max, 'f', 2);
          matrix->getSpinbox(r,c)->setToolTip(tooltip);
        }
      }
    }
    else {
      ROS_ERROR_STREAM("Wrong dimension for matrix parameter " << name_);
    }
  }

  template <typename MultiArrayMsg_>
  void writeMatrixToMessage(MatrixSpinBox * matrix, MultiArrayMsg_ & msg) {
    // Add matrix data dimension
    if(matrix->rows()*matrix->cols() > 0) {
      msg.layout.dim.resize(2);
      msg.layout.dim[0].label = "rows";
      msg.layout.dim[0].size = matrix->rows();
      msg.layout.dim[0].stride = matrix->rows()*matrix->cols();
      msg.layout.dim[1].label = "cols";
      msg.layout.dim[1].size = matrix->cols();
      msg.layout.dim[1].stride = matrix->cols();

      msg.data.resize(matrix->rows()*matrix->cols());
      for( std::size_t r = 0; r < matrix->rows(); ++r ) {
        for( std::size_t c = 0; c < matrix->cols(); ++c ) {
          msg.data[r*matrix->cols() + c] = static_cast<std::int64_t>(matrix->getSpinbox(r,c)->value());
        }
      }
    }
    else {
      ROS_ERROR_STREAM("Matrix size is zero.");
    }
  }

  public slots:

  void pushButtonChangeParamPressed() {
    ROS_INFO_STREAM("Change parameter " << name_ << ".");

    parameter_handler_msgs::GetFloatingPointParameterRequest reqGet;
    parameter_handler_msgs::GetFloatingPointParameterResponse resGet;

    reqGet.name = name_;
    getParameterClient_->call(reqGet, resGet);

    parameter_handler_msgs::SetFloatingPointParameterRequest req;
    parameter_handler_msgs::SetFloatingPointParameterResponse res;

    req.name = name_;
    writeMatrixToMessage<std_msgs::Float64MultiArray>(matrixSpinBoxParamValue, req.value);

    if(!setParameterClient_->call(req, res)) {
      ROS_ERROR_STREAM("Could not set parameter " << name_);
    }

  }

  void refreshParam() {
    parameter_handler_msgs::GetFloatingPointParameterRequest req;
    parameter_handler_msgs::GetFloatingPointParameterResponse res;

    req.name = name_;
    if (getParameterClient_->exists()) {
      if (getParameterClient_->call(req, res)) {
        if(res.success) {
          readMatrixParamFromMessage<parameter_handler_msgs::GetFloatingPointParameterResponse>(matrixSpinBoxParamValue, res);
        }
      }
      else {
        ROS_WARN_STREAM("Could not get parameter " << name_);
      }
    }
  }
};
