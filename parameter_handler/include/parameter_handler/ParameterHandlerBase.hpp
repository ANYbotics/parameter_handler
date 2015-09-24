/*
 * ParameterHandlerBase.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: gech
 */

#pragma once

#include <parameter_handler/ParameterInterface.hpp>
#include <string>

namespace parameter_handler {

class ParameterHandlerBase
{
 public:
  ParameterHandlerBase() {

  }
  virtual ~ParameterHandlerBase() {

  }

  virtual bool addParam(const std::string& name, ParameterInterface& param) = 0;
  //virtual bool setParamValue(const std::string& name, const ParameterInterface::type& value) = 0;
  virtual bool getParam(const std::string& name, ParameterInterface& param) = 0;
};

} /* namespace parameter_handler */
