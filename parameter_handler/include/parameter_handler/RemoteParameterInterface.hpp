/*!
 * @file    RemoteParameterInterface.hpp
 * @author  Francisco Giraldez
 * @date    20.03.19
 */

#pragma once

#include <parameter_handler/ParameterValue.hpp>

#include <string>
#include <memory>
#include <typeindex>

namespace parameter_handler {

class RemoteParameterInterface : ParameterInterface {
 protected:
  RemoteParameterInterface(const std::type_index& type, const internal::ParameterValuePtr& value, const std::string& name = std::string()) :
    type_(type),
    value_(value),
    name_(name)
  {
  }
 public:
  RemoteParameterInterface() :
    type_(typeid(void))
  {
  }

  RemoteParameterInterface(const RemoteParameterInterface& other) :
    type_(other.type_),
    value_(other.value_),
    name_(other.name_)
  {
  }
  virtual ~RemoteParameterInterface() {}

 public:
  template <typename ValueType_>
  void setMinValue(const ValueType_& value) = delete;

  template<typename ValueType_>
  void setMaxValue(const ValueType_& value) = delete;

  template<typename ValueType_>
  void setDefaultValue(const ValueType_& value) = delete;
  
};

} // namespace
