/*!
 * @file    RemoteParameterInterface.hpp
 * @author  Francisco Giraldez
 * @date    20.03.19
 */

#pragma once

#include "parameter_handler/RemoteParameterInterface.hpp"
#include "parameter_handler/ParameterValueInterface.hpp"
#include "parameter_handler/ParameterValue.hpp"

#include "tinyxml_tools/tinyxml_tools.hpp"

#include "message_logger/message_logger.hpp"

namespace parameter_handler {

template<typename ValueType_>
class RemoteParameter : public RemoteParameterInterface
{
public:
  RemoteParameter(const ValueType_& value = ValueType_()) :
    RemoteParameterInterface(typeid(ValueType_),
                       internal::ParameterValuePtr(new internal::ParameterValue<ValueType_>()))
  {
  }

  RemoteParameter(const std::string& name) :
  RemoteParameterInterface(typeid(ValueType_),
                       internal::ParameterValuePtr(new internal::ParameterValue<ValueType_>()),
                       name)
  {
  }

  RemoteParameter(const std::string& name, const ValueType_& value) :
    RemoteParameterInterface(typeid(ValueType_),
                       internal::ParameterValuePtr(new internal::ParameterValue<ValueType_>()),
                       name)
  {
    getValuePtr()->setValue(value);
  }


  RemoteParameter(const RemoteParameter<ValueType_>& other) :
    RemoteParameterInterface(other)
  {
  }

  RemoteParameter(const RemoteParameterInterface& other) :
    RemoteParameterInterface(other)
  {
    if (type_ != typeid(ValueType_))
      throw std::runtime_error("RemoteParameter value type mismatch");
  }

  virtual ~RemoteParameter()  { }


  const ValueType_& getValue() const {
    return getValuePtr()->getValue();
  }

  const ValueType_& getDefaultValue() const {
    return getValuePtr()->getDefaultValue();
  }

  const ValueType_& getMinValue() const {
    return getValuePtr()->getMinValue();
  }

  const ValueType_& getMaxValue() const {
    return getValuePtr()->getMaxValue();
  }

  void setValue(const ValueType_& value) {
    getValuePtr()->setValue(value);
    notifyObservers();
  }

  template< typename V_ = ValueType_>
  void setValue(const typename V_::Scalar& value, unsigned int row, unsigned int col = 0,
                typename std::enable_if< std::is_base_of< Eigen::MatrixBase<V_>, V_ >::value>::type* = 0 /* is_eigen */ )
  {
    getValuePtr()->setValue(value, row, col);
    notifyObservers();
  }

  void resetToDefault() {
    getValuePtr()->setValue(getValuePtr()->getDefaultValue());
    notifyObservers();
  }

  // bool load(const std::string& filename) {
  //   tinyxml_tools::DocumentHandleXML doc;
  //   if( doc.create(filename, tinyxml_tools::DocumentMode::READ)) {
  //     return doc.read(getName(), *this);
  //   }
  //   return false;
  // }

  // bool store(const std::string& filename, bool append = false) const {
  //   tinyxml_tools::DocumentHandleXML doc;
  //   auto mode = append ? tinyxml_tools::DocumentMode::APPEND : tinyxml_tools::DocumentMode::WRITE;
  //   if( doc.create(filename, mode) && doc.write(getName(), *this) ) {
  //     return doc.save();
  //   }
  //   return false;
  // }

  // bool load(const tinyxml_tools::DocumentHandleXML& doc) {
  //   return doc.read(getName(), *this);
  // }

  // bool store(tinyxml_tools::DocumentHandleXML& doc) const {
  //   return doc.write(getName(), *this);
  // }

protected:
  std::shared_ptr<internal::ParameterValue<ValueType_> > getValuePtr() {
    return std::static_pointer_cast<internal::ParameterValue<ValueType_> >(value_);
  }

  std::shared_ptr<const internal::ParameterValue<ValueType_> > getValuePtr() const {
    return std::static_pointer_cast<const internal::ParameterValue<ValueType_> >(value_);
  }

};

template<typename T>
struct is_parameter : std::false_type { };

template<typename U>
struct is_parameter<parameter_handler::RemoteParameter<U>> : std::true_type { };

template<typename T, typename ValueType>
struct is_parameter_of_type : std::false_type { };

template<typename U, typename ValueType>
struct is_parameter_of_type<parameter_handler::RemoteParameter<U>, ValueType> : std::is_same<U, ValueType> { };


}
