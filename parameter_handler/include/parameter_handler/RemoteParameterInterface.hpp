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

class RemoteParameterInterface {
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
  const std::string& getName() const {
    return name_;
  }
  void setName(const std::string& name) {
    name_ = name;
  }

  const std::type_index& getType() const {
    return type_;
  }

  template<typename ValueType_>
  const ValueType_& getValue() const
  {
    if (type_ == typeid(ValueType_)) {
      return std::static_pointer_cast<internal::ParameterValue<ValueType_> >(value_)->getValue();
    }
    else {
      throw std::runtime_error("Parameter value type mismatch");
    }
  };

  template<typename ValueType_>
  const ValueType_& getMinValue() const
  {
    if (type_ == typeid(ValueType_)) {
      return std::static_pointer_cast<internal::ParameterValue<ValueType_> >(value_)->getMinValue();
    }
    else {
      throw std::runtime_error("Parameter value type mismatch");
    }
  }
  template<typename ValueType_>
  const ValueType_& getMaxValue() const
  {
    if (type_ == typeid(ValueType_)) {
      return std::static_pointer_cast<internal::ParameterValue<ValueType_> >(value_)->getMaxValue();
    }
    else {
      throw std::runtime_error("Parameter value type mismatch");
    }
  }
  template<typename ValueType_>
  const ValueType_& getDefaultValue() const
  {
    if (type_ == typeid(ValueType_)) {
      return std::static_pointer_cast<internal::ParameterValue<ValueType_> >(value_)->getDefaultValue();
    }
    else {
      throw std::runtime_error("Parameter value type mismatch");
    }
  }

  template<typename ValueType_>
  void setValue(const ValueType_& value)
  {
    if (type_ == typeid(ValueType_)) {
      std::static_pointer_cast<internal::ParameterValue<ValueType_> >(value_)->setValue(value);
      notifyObservers();
    }
    else {
      throw std::runtime_error("Parameter value type mismatch");
    }
  }

  RemoteParameterInterface clone() const {
    return RemoteParameterInterface(type_, value_->clone(), name_);
  }

  RemoteParameterInterface& operator=(const RemoteParameterInterface& other) {
    type_ = other.type_;
    value_ = other.value_;
    name_ = other.name_;
    return *this;
  }

  void notifyObservers() {
    for(auto observer: value_->getObservers()) { observer->parameterChanged(*this); };
  }

  void addObserver(ParameterObserverInterface * observer) {
    value_->addObserver(observer);
  }

  void removeObserver(ParameterObserverInterface * observer) {
    value_->removeObserver(observer);
  }

 protected:
  std::type_index type_;
  internal::ParameterValuePtr value_;
  std::string name_;
};


} // namespace
