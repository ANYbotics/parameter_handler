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
#include "parameter_handler/parameter_traits.hpp"
#include <limits>

namespace parameter_handler {

template<typename ValueType_>
class Parameter : public ParameterInterface<ValueType_>
{
 public:
  typedef ParameterInterface<ValueType_> Base;
  typedef ValueType_ ValueType;

  using traits = internal::parameter_traits<ValueType_>;

  Parameter()
      : param_(std::numeric_limits<ValueType_>::min()),
        paramMin_(-std::numeric_limits<ValueType_>::max()),
        paramMax_(std::numeric_limits<ValueType_>::max()),
        default_(std::numeric_limits<ValueType_>::min())
  { }
  Parameter(const Parameter& other) = default;

  Parameter(const ValueType_& param)
      : param_(param),
        default_(param)
  { }

  Parameter(const ValueType_& param, const ValueType_& min, const ValueType_& max)
      : param_(param),
        paramMin_(min),
        paramMax_(max),
        default_(param)
  { }

  virtual ~Parameter()
  { }

  virtual const std::string& getName() const {
    return name_;
  }
  virtual const ValueType_& getMinValue() const
  {
    return paramMin_;
  }
  virtual const ValueType_& getMaxValue() const
  {
    return paramMax_;
  }
  virtual const ValueType_& getDefaultValue() const
  {
    return default_;
  }
  virtual const ValueType_& getCurrentValue() const
  {
    return param_;
  }


  virtual void setName(const std::string& name) {
    name_ = name;
  }
  virtual void setCurrentValue(const ValueType_& value)
  {
    param_ = value;
  }
  virtual void setMinValue(const ValueType_& value)
  {
    paramMin_ = value;
  }
  virtual void setMaxValue(const ValueType_& value)
  {
    paramMax_ = value;
  }
  virtual void setDefaultValue(const ValueType_& value)
  {
    default_ = value;
  }

 protected:
  ValueType_ param_;
  ValueType_ default_;
  ValueType_ paramMin_;
  ValueType_ paramMax_;
  std::string name_;

};

using ParameterDouble = Parameter<double>;
using ParameterFloat = Parameter<float>;
using ParameterInt = Parameter<int>;
using ParameterBool = Parameter<bool>;

}

