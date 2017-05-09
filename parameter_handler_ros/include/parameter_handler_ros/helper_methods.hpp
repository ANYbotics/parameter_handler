/*
 * helper_methods.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Gabriel Hottiger
 */

#include "parameter_handler_ros/type_macros.hpp"
#include "parameter_handler/ParameterInterface.hpp"

#include <ros/ros.h>
#include <Eigen/Core>

#pragma once

namespace parameter_handler_ros {

template<typename T1>
bool isType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    return true;
  } else {
    return false;
  }
}

template <typename T1, typename T2, typename... Tn>
bool isType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    return true;
  } else {
    return isType<T2, Tn...>(param);
  }
}

namespace internal {

template <typename ScalarType_, typename MultiArrayMsg_>
bool writeScalarToMessage(const ScalarType_ & scalar, MultiArrayMsg_ & msg) {
  using PrimType = typename MultiArrayMsg_::_data_type::value_type;

  // Add scalar data dimension
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "data";
  msg.layout.dim[0].size = 1;
  msg.layout.dim[0].stride = 1;
  msg.data.resize(1);
  msg.data[0] = static_cast<PrimType>(scalar);

  return true;
}

template <typename MatrixType_, typename MultiArrayMsg_>
bool writeMatrixToMessage(const MatrixType_ & matrix, MultiArrayMsg_ & msg) {
  using PrimType = typename MultiArrayMsg_::_data_type::value_type;

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
        msg.data[r*matrix.cols() + c] = static_cast<PrimType>(matrix(r,c));
      }
    }
  }
  else {
    ROS_ERROR_STREAM("Matrix size is zero.");
    return false;
  }

  return true;
}

}

template <typename GetParamResponse_>
bool writeScalarParamToMessage(const parameter_handler::ParameterInterface & param, GetParamResponse_ & msg) {
  return false;
}

template <typename GetParamResponse_, typename T1, typename... Tn>
bool writeScalarParamToMessage(const parameter_handler::ParameterInterface & param, GetParamResponse_ & msg) {
  if(isType<T1>(param)) {
    bool success = true;
    success = success && internal::writeScalarToMessage(param.getValue<T1>(), msg.value_current);
    success = success && internal::writeScalarToMessage(param.getMinValue<T1>(), msg.value_min);
    success = success && internal::writeScalarToMessage(param.getMaxValue<T1>(), msg.value_max);
    success = success && internal::writeScalarToMessage(param.getDefaultValue<T1>(), msg.value_default);
    return success;
  }

  return writeScalarParamToMessage<GetParamResponse_, Tn...>(param, msg);
}

template <typename GetParamResponse_>
bool writeMatrixParamToMessage(const parameter_handler::ParameterInterface & param, GetParamResponse_ & msg) {
  return false;
}

template <typename GetParamResponse_, typename T1, typename... Tn>
bool writeMatrixParamToMessage(const parameter_handler::ParameterInterface & param, GetParamResponse_ & msg) {
  if(isType<T1>(param)) {
    bool success = true;
    success = success && internal::writeMatrixToMessage(param.getValue<T1>(), msg.value_current);
    success = success && internal::writeMatrixToMessage(param.getMinValue<T1>(), msg.value_min);
    success = success && internal::writeMatrixToMessage(param.getMaxValue<T1>(), msg.value_max);
    success = success && internal::writeMatrixToMessage(param.getDefaultValue<T1>(), msg.value_default);
    return success;
  }

  return writeMatrixParamToMessage<GetParamResponse_, Tn...>(param, msg);
}

template<typename MultiArrayMsg_>
bool readScalarParamFromMessage(parameter_handler::ParameterInterface & param, const MultiArrayMsg_ & msg) {
  return false;
}

template<typename MultiArrayMsg_, typename T1, typename... Tn>
bool readScalarParamFromMessage(parameter_handler::ParameterInterface & param, const MultiArrayMsg_ & msg) {
  if(isType<T1>(param)) {
    // Handle scalar types
    if( (msg.layout.dim.size() == 1 && msg.layout.dim[0].size == 1) ||
        (msg.layout.dim.size() == 2 && msg.layout.dim[0].size == 1 && msg.layout.dim[1].size == 1 ) ) {
      param.setValue(static_cast<T1>(msg.data[0]));
      return true;
    }
    else {
      ROS_ERROR_STREAM("Wrong dimension for scalar parameter " << param.getName());
      return false;
    }
  }

  return readScalarParamFromMessage<MultiArrayMsg_, Tn...>(param, msg);
}

template<typename MultiArrayMsg_>
bool readMatrixParamFromMessage(parameter_handler::ParameterInterface & param, const MultiArrayMsg_ & msg) {
  return false;
}

template<typename MultiArrayMsg_, typename T1, typename... Tn>
bool readMatrixParamFromMessage(parameter_handler::ParameterInterface & param, const MultiArrayMsg_ & msg) {
  if(isType<T1>(param)) {
    // Handle eigen matrices
    if(msg.layout.dim.size() == 1 || msg.layout.dim.size() == 2) {
      int rows = msg.layout.dim[0].size;
      int cols = (msg.layout.dim.size() == 2) ? msg.layout.dim[1].size : 1;
      if(param.getValue<T1>().rows() == rows  && param.getValue<T1>().cols() == cols )
      {
        T1 m = Eigen::Matrix<typename T1::Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows, cols);
        for( int r = 0; r < rows; ++r ) {
          for( int c = 0; c < cols; ++c ) {
            m(r,c) = static_cast<typename T1::Scalar>(msg.data[r*cols + c]);
          }
        }
        param.setValue(m);
        return true;
      }
      else {
        ROS_ERROR_STREAM("Wrong matrix size for parameter " << param.getName());
        return false;
      }
    }
    else {
      ROS_ERROR_STREAM("Wrong dimension for matrix parameter " << param.getName());
      return false;
    }
  }

  return readMatrixParamFromMessage<MultiArrayMsg_, Tn...>(param, msg);
}

}
