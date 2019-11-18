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
 * ParameterInterfaceList.hpp
 *
 *  Created on: Apr 23, 2015
 *      Author: C. Dario Bellicoso, Christian Gehring
 */

#pragma once

#include <message_logger/message_logger.hpp>
#include <parameter_handler/ParameterHandlerBase.hpp>
#include <parameter_handler/ParameterInterface.hpp>
#include <parameter_handler/helper_methods.hpp>

#include <boost/any.hpp>
#include <iostream>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace parameter_handler_std {

class ParameterHandlerStd : public parameter_handler::ParameterHandlerBase {
 public:
  using ParameterList = std::unordered_map<std::string, parameter_handler::ParameterInterface>;

 public:
  ParameterHandlerStd() = default;
  ~ParameterHandlerStd() override = default;

  bool addParam(const std::string& name, parameter_handler::ParameterInterface& param, bool verbose) override;

  bool addParam(parameter_handler::ParameterInterface& param, bool verbose) override;
  bool getParam(const std::string& name, parameter_handler::ParameterInterface& param) override;

  void parameterChanged(const parameter_handler::ParameterInterface& param) override;

  bool storeParams(const std::string& filename, bool append) const override;
  bool loadParams(const std::string& filename) override;

 protected:
  ParameterList params_;
  std::mutex mutexParams_;
};

}  // namespace parameter_handler_std
