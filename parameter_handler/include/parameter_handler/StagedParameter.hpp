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
 * StagedParameter.hpp
 *
 *  Created on: Mar 16, 2022
 *      Author: Magnus Gaertner
 */

#pragma once

#include "parameter_handler/Parameter.hpp"

namespace parameter_handler {

/**
 * Opposed to Parameter, StagedParameter provides a staged frontend to read values.
 * I.e a setValue() is only visible in getValue once updateValue is called.
 */
template <typename ValueType_>
class StagedParameter : public Parameter<ValueType_> {
 public:
  using BASE = Parameter<ValueType_>;

  StagedParameter(const ValueType_& value = ValueType_()) : BASE{value} { value_ = BASE::getValue(); }

  StagedParameter(const ValueType_& value, const ValueType_& min, const ValueType_& max) : BASE{value, min, max} {
    value_ = BASE::getValue();
  }

  StagedParameter(const std::string& name, const ValueType_& value, const ValueType_& min, const ValueType_& max)
      : BASE{name, value, min, max} {
    value_ = BASE::getValue();
  }

  StagedParameter(const StagedParameter<ValueType_>& other) : BASE(other) { value_ = BASE::getValue(); }

  explicit StagedParameter(const ParameterInterface& other) : BASE(other) {
    if (BASE::getType() != typeid(ValueType_)) {
      throw std::runtime_error("Parameter value type mismatch");
    }
  }

  ~StagedParameter() override = default;

  ValueType_ getValue() const { return value_; }

  void updateValue() { value_ = BASE::getValue(); }

 private:
  ValueType_ value_;
};

}  // namespace parameter_handler
