/**
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, C. Dario Bellicoso, Christian Gehring, Ralf Kaestner
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
 * ParameterValue.hpp
 *
 *  Created on: Sep 24, 2015
 *      Author: Christian Gehring, Ralf Kaestner
 */

#pragma once

#include "parameter_handler/ParameterValueInterface.hpp"
#include "parameter_handler/ParameterValueTraits.hpp"
#include <limits>

namespace parameter_handler {
namespace internal {

template<typename ValueType_>
class ParameterValue: public ParameterValueInterface {
 public:
  typedef ValueType_ ValueType;
 public:
  ParameterValue()
  {
    internal::ParameterValueTraits<ParameterValue<ValueType_> >::init(*this);
  }

  ParameterValue(const ValueType_& param, const ValueType_& min, const ValueType_& max)
      : value_(param),
        valueMin_(min),
        valueMax_(max),
        default_(param)
  { }

  virtual ~ParameterValue() {

  }

  ParameterValuePtr clone() const {
    return ParameterValuePtr(new ParameterValue<ValueType_>(*this));
  }

  const ValueType_& getMinValue() const
  {
    return valueMin_;
  }
  const ValueType_& getMaxValue() const
  {
    return valueMax_;
  }
  const ValueType_& getDefaultValue() const
  {
    return default_;
  }
  const ValueType_& getValue() const
  {
    return value_;
  }

  void setValue(const ValueType_& value)
  {
    value_ = internal::ParameterValueTraits<ParameterValue<ValueType_> >::setValue(*this, value);
  }
  void setMinValue(const ValueType_& value)
  {
    valueMin_ = value;
  }
  void setMaxValue(const ValueType_& value)
  {
    valueMax_ = value;
  }
  void setDefaultValue(const ValueType_& value)
  {
    default_ = internal::ParameterValueTraits<ParameterValue<ValueType_> >::setValue(*this, value);
  }

  protected:
   ValueType_ value_;
   ValueType_ default_;
   ValueType_ valueMin_;
   ValueType_ valueMax_;
};

} // namspace internal
} // namespace
