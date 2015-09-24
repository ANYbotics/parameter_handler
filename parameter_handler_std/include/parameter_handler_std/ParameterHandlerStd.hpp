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

#include <parameter_handler/ParameterHandlerBase.hpp>
#include <parameter_handler/ParameterInterface.hpp>
#include <roco/log/log_messages.hpp>

#include <vector>
#include <unordered_map>
#include <iostream>
#include <boost/any.hpp>
#include <mutex>

namespace parameter_handler_std {

class ParameterHandlerStd : public parameter_handler::ParameterHandlerBase {
 public:
  typedef std::unordered_map<std::string, parameter_handler::ParameterInterface*> ParameterList;
 public:
  ParameterHandlerStd();
  virtual ~ParameterHandlerStd();

  virtual bool addParam(const std::string& name, parameter_handler::ParameterInterface& param) {
    std::lock_guard<std::mutex> lock(mutexParams_);

    auto paramIterator = params_.find(name);

    if (!(paramIterator == params_.end())) {
      ROCO_WARN_STREAM("Key '" << name << "' was already inserted in ParameterInterface list.");
      return false;
    }

    params_.insert( { name, &param });
    return true;
  }

  virtual bool getParam(const std::string& name, parameter_handler::ParameterInterface& param) {


    std::lock_guard<std::mutex> lock(mutexParams_);

    auto paramIterator = params_.find(name);

    if (paramIterator == params_.end()) {
      ROCO_INFO_STREAM("Key '" << name << "' was not found.");
      return false;
    }
    try {
      param = *paramIterator->second;
    }
    catch (...) {
      //const std::type_info &ti = paramIterator->second.type();
      //ROCO_WARN_STREAM("Requested ParameterInterface '" << name << "' has type " << ti.name() << " instead of " << typeid(parameter_handler::ParameterInterface<ValueType_>).name());
      return false;
    }
    return true;

  }

 protected:
  ParameterList params_;
  std::mutex mutexParams_;

};

} /* namespace */
