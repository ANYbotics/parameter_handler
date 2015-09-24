/*
 * ParameterInterface.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: gech
 */

#pragma once

#include <parameter_handler/ParameterValue.hpp>
#include <parameter_handler/ParameterValueImpl.hpp>
#include <string>
#include <memory>

namespace parameter_handler {


class ParameterInterface {
 public:
  typedef std::shared_ptr<ParameterValue> ParameterValuePtr;
 public:
  ParameterInterface() {}
  virtual ~ParameterInterface() {}

  const std::string& getName() const {
    return name_;
  }
  void setName(const std::string& name) {
    name_ = name;
  }

  template<typename ValueType_>
  const ValueType_& getCurrentValue() const;
  template<typename ValueType_>
  const ValueType_& getMinValue() const;
  template<typename ValueType_>
  const ValueType_& getMaxValue() const;
  template<typename ValueType_>
  const ValueType_& getDefaultValue() const;
  template<typename ValueType_>
  void setCurrentValue(const ValueType_& value);
  template<typename ValueType_>
  void setMinValue(const ValueType_& value);
  template<typename ValueType_>
  void setMaxValue(const ValueType_& value);
  template<typename ValueType_>
  void setDefaultValue(const ValueType_& value);

 protected:
  ParameterValuePtr value_;
  std::string name_;

};

} // namespace
