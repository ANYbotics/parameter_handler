/*
 * ParameterValueImpl.hpp
 *
 *  Created on: Sep 24, 2015
 *      Author: gech
 */

#pragma once

#include "parameter_handler/ParameterValue.hpp"

#include <limits>

namespace parameter_handler {

template<typename ValueType_>
class ParameterValueImpl: public ParameterValue {
 public:
  typedef ValueType_ ValueType;
 public:
  ParameterValueImpl() :
    param_(std::numeric_limits<ValueType_>::min()),
    default_(param_),
    paramMin_(-std::numeric_limits<ValueType_>::max()),
    paramMax_(std::numeric_limits<ValueType_>::max())
  {

  }

  ParameterValueImpl(const ValueType_& param, const ValueType_& min, const ValueType_& max)
      : param_(param),
        paramMin_(min),
        paramMax_(max),
        default_(param)
  { }

  virtual ~ParameterValueImpl() {

  }

  ParameterValueImpl<ValueType_>* clone() const {
    return new ParameterValueImpl<ValueType_>();
  }

  const ValueType_& getMinValue() const
  {
    return paramMin_;
  }
  const ValueType_& getMaxValue() const
  {
    return paramMax_;
  }
  const ValueType_& getDefaultValue() const
  {
    return default_;
  }
  const ValueType_& getCurrentValue() const
  {
    return param_;
  }

  void setCurrentValue(const ValueType_& value)
  {
    param_ = value;
  }
  void setMinValue(const ValueType_& value)
  {
    paramMin_ = value;
  }
  void setMaxValue(const ValueType_& value)
  {
    paramMax_ = value;
  }
  void setDefaultValue(const ValueType_& value)
  {
    default_ = value;
  }

  protected:
   ValueType_ param_;
   ValueType_ default_;
   ValueType_ paramMin_;
   ValueType_ paramMax_;
};


} // namespace
