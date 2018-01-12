/*!
 * @file	  traits.hpp
 * @author	Gabriel Hottiger
 * @date	  Jan, 2018
 */

#pragma once

// parameter_handler
#include "parameter_handler/Parameter.hpp"

// tinyxml
#include <tinyxml.h>

// tinyxml_tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// Eigen
#include <Eigen/Core>

// STL
#include <string>
#include <array>
#include <vector>

namespace tinyxml_tools {

namespace traits {

//! Element trait for parameters
template<typename T>
struct element<parameter_handler::Parameter<T>> {
  // Types
  using type = parameter_handler::Parameter<T>;
  using stringType = typename element<T>::stringType;

  // Default values
  static const stringType getDefaultName() { return element<T>::getDefaultName(); }
  static type getDefaultValue() { return element<T>::getDefaultValue(); }

  // Read trait
  static bool read(type& value, const TiXmlElement* element, const stringType& name, const type& def) {
    T v(value.getValue());
    bool success = traits::element<T>::read(v, element, name, value.getDefaultValue());
    value.setValue(v);
    return success;
  }

  // Write trait
  static void write(const type& value, TiXmlElement* const element, const stringType& name) {
    traits::element<T>::write(value.getValue(), element, name);
  }
};

} // traits

} // tinyxml_tools