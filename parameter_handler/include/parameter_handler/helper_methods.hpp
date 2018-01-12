/*
 * helper_methods.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Gabriel Hottiger
 */

// parameter handler
#include "parameter_handler/type_macros.hpp"
#include "parameter_handler/ParameterInterface.hpp"

// message logger
#include "message_logger/message_logger.hpp"

#pragma once

namespace parameter_handler {

template<typename T1>
bool isType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    return true;
  } else {
    return false;
  }
}

template <typename T1, typename T2, typename... Tn>
bool isType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    return true;
  } else {
    return isType<T2, Tn...>(param);
  }
}

template<typename T1>
void printType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    MELO_INFO_STREAM( "Changed parameter " << param.getName() << " to :" << param.getValue<T1>() );
  }
}

template <typename T1, typename T2, typename... Tn>
void printType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    printType<T1>(param);
  } else {
    printType<T2, Tn...>(param);
  }
}

template<typename T1>
bool storeType(const parameter_handler::ParameterInterface & param, TiXmlElement* rootElement) {
  if( param.getType() == typeid(T1) ){
    return tinyxml_tools::writeParameter(param.getName(), param.getValue<T1>(), rootElement);
  }
  return false;
}

template <typename T1, typename T2, typename... Tn>
bool storeType(const parameter_handler::ParameterInterface & param, TiXmlElement* rootElement) {
  if( param.getType() == typeid(T1) ){
    return storeType<T1>(param, rootElement);
  } else {
    return storeType<T2, Tn...>(param, rootElement);
  }
}

template<typename T1>
bool loadType(parameter_handler::ParameterInterface & param, TiXmlElement* rootElement) {
  if( param.getType() == typeid(T1) ){
    T1 v(param.getValue<T1>());
    bool success =  tinyxml_tools::loadParameter(param.getName(), v, rootElement);
    param.setValue<T1>(v);
    return success;
  }
  return false;
}

template <typename T1, typename T2, typename... Tn>
bool loadType(parameter_handler::ParameterInterface & param, TiXmlElement* rootElement) {
  if( param.getType() == typeid(T1) ){
    return loadType<T1>(param, rootElement);
  } else {
    return loadType<T2, Tn...>(param, rootElement);
  }
}

}
