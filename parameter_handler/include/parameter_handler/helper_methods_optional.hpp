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
 * parameter_handler_optional.hpp
 *
 *  Created on: Mar 18, 2022
 *      Author: Magnus Gaertner
 */
#pragma once
#include "parameter_handler/parameter_handler.hpp"

#include <boost/pfr.hpp>

namespace parameter_handler {

template <typename StructOfParametersT>
bool registerParameterStruct(StructOfParametersT& parameters) {
  bool success = true;
  boost::pfr::for_each_field(parameters, [&success](auto& parameter) {
    success &= parameter_handler::handler->addParam(static_cast<ParameterInterface&>(parameter));
  });
  return success;
}

template <typename StructOfParametersT>
bool removeParametersStruct(StructOfParametersT& parameters) {
  bool success = true;
  boost::pfr::for_each_field(parameters,
                             [&success](auto& parameter) { success &= parameter_handler::handler->removeParam(parameter.getName()); });
  return success;
}

}  // namespace parameter_handler
