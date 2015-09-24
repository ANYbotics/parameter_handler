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

#include "parameter_handler/ParameterInterface.hpp"
#include <roco/log/log_messages.hpp>

#include <vector>
#include <unordered_map>
#include <iostream>
#include <boost/any.hpp>

namespace parameter_handler {

class ParameterHandlerStd {
 public:
  typedef std::unordered_map<std::string, boost::any> ParameterInterfaceList;
 public:
  ParameterHandlerStd();
  virtual ~ParameterHandlerStd();


  template <typename ValueType_>
  bool addParam(const std::string& name, ParameterInterface<ValueType_>& param) {
    auto paramIterator = params_.find(name);

    if (!(paramIterator == params_.end())) {
      ROCO_WARN_STREAM("Key '" << name << "' was already inserted in ParameterInterface list.");
      return false;
    }

    params_.insert( { name, boost::any(&param) });
    return true;
  }

  template <typename ValueType_>
  bool setParamValue(std::string name, const ValueType_& value) {
    auto paramIterator = params_.find(name);

    if (paramIterator == params_.end()) {
      ROCO_WARN_STREAM("Key '" << name << "' was not found in ParameterInterface list.");
      return false;
    }

    boost::any_cast<ParameterInterface<ValueType_>*>(paramIterator->second)->setCurrentValue(value);

    return true;
  }


  template <typename ValueType_>
  bool getParamValue(std::string name, ValueType_& value) {
    auto paramIterator = params_.find(name);

    if (paramIterator == params_.end()) {
      ROCO_WARN_STREAM("Key '" << name << "' was not found in ParameterInterface list.");
      return false;
    }

    value = boost::any_cast<ParameterInterface<ValueType_>*>(paramIterator->second)->getCurrentValue();

    return true;
  }


  template <typename ValueType_>
  bool getParam(const std::string& name,  ParameterInterface<ValueType_>*& param) {
    auto paramIterator = params_.find(name);

    if (paramIterator == params_.end()) {
      ROCO_INFO_STREAM("Key '" << name << "' was not found.");
      return false;
    }
    try {
      param = boost::any_cast<ParameterInterface<ValueType_>*>(paramIterator->second);
    }
    catch (...) {
      std::cout << "catch param name: " << boost::any_cast<std::shared_ptr<ParameterInterface<ValueType_>>>(paramIterator->second)->getName() << std::endl;
      const std::type_info &ti = paramIterator->second.type();
      ROCO_WARN_STREAM("Requested ParameterInterface '" << name << "' has type " << ti.name() << " instead of " << typeid(ParameterInterface<ValueType_>).name());
      return false;
    }
    return true;

  }

  const ParameterInterfaceList getList() {
    return params_;
  }

 private:
  ParameterInterfaceList params_;

};

} /* namespace */
