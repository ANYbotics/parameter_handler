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
 * ParameterHandlerStd.cpp
 *
 *  Created on: Jul 3, 2015
 *      Author: Dario Bellicoso, Christian Gehring
 */

#include "parameter_handler_std/ParameterHandlerStd.hpp"

#include "tinyxml_tools/tinyxml_tools.hpp"
#include <tinyxml.h>

namespace parameter_handler_std {

bool ParameterHandlerStd::addParam(const std::string& name, parameter_handler::ParameterInterface& param, bool verbose) {
  std::lock_guard<std::mutex> lock(mutexParams_);

  // Set the name of the parameter and add observer
  param.setName(name);
  if(verbose) { param.addObserver(this); }

  auto paramIterator = params_.find(name);

  if (!(paramIterator == params_.end())) {
    MELO_WARN_STREAM("Key '" << name << "' was already inserted in ParameterInterface list. Overwriting reference!");
    paramIterator->second = param;
    return true;
  }

  params_.insert( { name, param });
  params_[name].notifyObservers();

  return true;
}

bool ParameterHandlerStd::addParam(parameter_handler::ParameterInterface& param, bool verbose) {
  return addParam(param.getName(), param, verbose);
}

bool ParameterHandlerStd::getParam(const std::string& name, parameter_handler::ParameterInterface& param) {


  std::lock_guard<std::mutex> lock(mutexParams_);

  auto paramIterator = params_.find(name);

  if (paramIterator == params_.end()) {
    MELO_INFO_STREAM("Key '" << name << "' was not found.");
    return false;
  }

  param = paramIterator->second;

  return true;

}

void ParameterHandlerStd::parameterChanged(const parameter_handler::ParameterInterface & param) {
  parameter_handler::printType<PH_TYPES>(param);
  return;
}

bool ParameterHandlerStd::storeParams(const std::string & filename) const {
  TiXmlDocument doc;
  auto * root = new TiXmlElement("Parameters");
  doc.LinkEndChild( new TiXmlDeclaration("1.0", "", "" ) );
  doc.LinkEndChild(root);
  bool success = true;
  for(const auto & param : params_) {
    success = param.second.store(root) && success;
  }
  return doc.SaveFile( filename ) && success;
}

bool ParameterHandlerStd::loadParams(const std::string & filename) {
  TiXmlDocument doc;
  TiXmlHandle docHandle(&doc), rootHandle(docHandle);
  if(!doc.LoadFile(filename) || !tinyxml_tools::getChildHandle(docHandle, rootHandle, "Root") || rootHandle.ToElement() == nullptr) {
    MELO_WARN_STREAM("Could not load document " << filename << ". Does tag 'Root' exist?");
    return false;
  }
  bool success = true;
  for(auto & param : params_) {
    success = param.second.load(rootHandle.ToElement()) && success;
  }
  return success;
}

} /* namespace */
