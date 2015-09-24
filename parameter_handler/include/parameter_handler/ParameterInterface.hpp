/*
 * ParameterInterface.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: gech
 */

#pragma once

#include <string>

template<typename ValueType_>
class ParameterInterface {
 public:
  ParameterInterface() {}
  virtual ~ParameterInterface() {}


  virtual const std::string& getName() const = 0;
  virtual const ValueType_& getCurrentValue() const = 0;
  virtual const ValueType_& getMinValue() const = 0;
  virtual const ValueType_& getMaxValue() const = 0;
  virtual const ValueType_& getDefaultValue() const = 0;

  virtual void setName(const std::string& name) = 0;
  virtual void setCurrentValue(const ValueType_& value) = 0;
  virtual void setMinValue(const ValueType_& value) = 0;
  virtual void setMaxValue(const ValueType_& value) = 0;
  virtual void setDefaultValue(const ValueType_& value) = 0;

};
