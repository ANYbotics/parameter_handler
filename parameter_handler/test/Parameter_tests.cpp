/*
 * Parameter_tests.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: Christian Gehring
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <parameter_handler/ParameterHandlerNone.hpp>
#include <parameter_handler/helper_methods.hpp>
#include <parameter_handler/parameter_handler.hpp>
#include "parameter_handler/Parameter.hpp"
#include "parameter_handler/StagedParameter.hpp"

using ParameterValueTypes = ::testing::Types<double, int, bool>;
using ::testing::_;
using ::testing::UnorderedElementsAre;

template <typename ValueType_>
class ParameterTest : public ::testing::Test {
 public:
  using Parameter = parameter_handler::Parameter<ValueType_>;
  using Value = ValueType_;
};

#ifndef ROS2_BUILD
TYPED_TEST_CASE(ParameterTest, ParameterValueTypes);
#else  // ROS2_BUILD
TYPED_TEST_SUITE(ParameterTest, ParameterValueTypes);  // NOLINT
#endif

#ifndef ROS2_BUILD
TEST(ParameterTest, setters) {  // NOLINT
#else
TYPED_TEST(ParameterTest, setters) {                   // NOLINT
#endif
  parameter_handler::Parameter<double> param;
  param.setValue(1.0);
  EXPECT_EQ(1.0, param.getValue());

  param.setMinValue(2.0);
  EXPECT_EQ(2.0, param.getMinValue());

  param.setMaxValue(3.0);
  EXPECT_EQ(3.0, param.getMaxValue());

  param.setDefaultValue(4.0);
  EXPECT_EQ(3.0, param.getDefaultValue());

  parameter_handler::ParameterInterface& paramInterface = param;
  EXPECT_EQ(paramInterface.getType(), typeid(double));
  EXPECT_NO_THROW(paramInterface.getValue<double>());  // NOLINT
  EXPECT_ANY_THROW(paramInterface.getValue<int>());    // NOLINT
}

#ifndef ROS2_BUILD
TEST(ParameterTest, testInt) {  // NOLINT
#else
TYPED_TEST(ParameterTest, testInt) {                   // NOLINT
#endif
  parameter_handler::Parameter<int> paramInt;
  paramInt.setValue(4);
  EXPECT_EQ(4, paramInt.getValue());
}

#ifndef ROS2_BUILD
TEST(ParameterTest, testStagedInt) {  // NOLINT
#else
TYPED_TEST(ParameterTest, testStagedInt) {             // NOLINT
#endif
  parameter_handler::StagedParameter<int> paramInt{4};
  EXPECT_EQ(4, paramInt.getValue());
  paramInt.setValue(2);
  EXPECT_EQ(4, paramInt.getValue());
  paramInt.updateValue();
  EXPECT_EQ(2, paramInt.getValue());
}

#ifdef USE_BOOST_PFR
struct HandledParameters {
  parameter_handler::Parameter<double> param1_{"param1", 0.0, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()};
  parameter_handler::Parameter<int> param2_{"param2", 0, -10, 10};
  parameter_handler::Parameter<bool> param3_{"param3", false, false, true};
};

/**
 * A small helper to check which parameters get registered.
 */
class ParameterHandlerTest : public parameter_handler::ParameterHandlerNone {
 public:
  bool addParam(parameter_handler::ParameterInterface& param, bool /*verbose*/) override {
    parameterNamesAdded_.push_back(param.getName());
    return true;
  }
  const auto& getAddedParameterNames() const { return parameterNamesAdded_; }

 private:
  std::list<std::string> parameterNamesAdded_;
};

TEST(ParameterTestOptional, testRegisterParameterStruct) {  // NOLINT
  std::shared_ptr<ParameterHandlerTest> parameterHandlerTest{new ParameterHandlerTest()};
  parameter_handler::handler = parameterHandlerTest;

  // Instantiate a struct of parameters and register it.
  HandledParameters parameters;
  parameter_handler::registerParameterStruct(parameters);

  EXPECT_THAT(parameterHandlerTest->getAddedParameterNames(), UnorderedElementsAre("param1", "param2", "param3"));
}

#endif
