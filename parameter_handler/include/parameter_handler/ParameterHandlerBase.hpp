/*
 * ParameterHandlerBase.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: gech
 */

#pragma once

#include <string>

namespace parameter_handler {

class ParameterHandlerBase
{
 public:
  ParameterHandlerBase() {

  }
  virtual ~ParameterHandlerBase() {

  }

  template<typename ParamType_>
  bool addParam(const std::string& name, ParamType_* param);

  template <typename ValueType_>
  bool setParamValue(std::string name, const ValueType_& value);

  template<typename ParamType_>
  bool getParam(const std::string& name, ParamType_& param);
};

} /* namespace parameter_handler */
