/**
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, C. Dario Bellicoso, Christian Gehring
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
 * ParameterImpl.hpp
 *
 *  Created on: Jul 3, 2015
 *      Author: Dario Bellicoso, Christian Gehring
 */

#pragma once

#include "parameter_handler/ParameterInterface.hpp"
#include "parameter_handler/ParameterValueImpl.hpp"
#include "parameter_handler/parameter_traits.hpp"
#include <limits>

namespace parameter_handler {

template<typename ParamType_>
class Parameter : public ParameterInterface
{
public:
  typedef typename ParamType_::ValueType ValueType;

  Parameter(const ParamType_& value = ParamType_()) {
    value_ = ParameterInterface::ParameterValuePtr(value.clone());
  }

  Parameter(const ValueType& value, const ValueType& min, const ValueType& max)
  {
    value_ = ParameterInterface::ParameterValuePtr(ParamType_().clone());
    getValuePtr()->setCurrentValue(value);
    getValuePtr()->setDefaultValue(value);
    getValuePtr()->setMinValue(min);
    getValuePtr()->setMaxValue(max);
  }

  virtual ~Parameter()  { }


  const ValueType& getCurrentValue() const {
    return getValuePtr()->getCurrentValue();
  }

  const ValueType& getDefaultValue() const {
    return getValuePtr()->getDefaultValue();
  }

  const ValueType& getMinValue() const {
    return getValuePtr()->getMinValue();
  }

  const ValueType& getMaxValue() const {
    return getValuePtr()->getMaxValue();
  }

  void setCurrentValue(const ValueType& value) {
    getValuePtr()->setCurrentValue(value);
  }

  void setDefaultValue(const ValueType& value) {
    getValuePtr()->setDefaultValue(value);
  }

  void setMinValue(const ValueType& value) {
    getValuePtr()->setMinValue(value);
  }

  void setMaxValue(const ValueType& value) {
    getValuePtr()->setMaxValue(value);
  }

protected:
  ParamType_* getValuePtr() const {
    return static_cast<ParamType_*>(value_.get());
  }

};

using ParameterDouble = Parameter<ParameterValueImpl<double>>;
using ParameterInt = Parameter<ParameterValueImpl<int>>;
//using ParameterFloat = Parameter<float>;
//using ParameterInt = Parameter<int>;
//using ParameterBool = Parameter<bool>;

}

