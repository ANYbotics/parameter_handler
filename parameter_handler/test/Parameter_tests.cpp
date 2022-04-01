/*
 * Parameter_tests.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: Christian Gehring
 */

#include <gtest/gtest.h>
#include "parameter_handler/Parameter.hpp"
#include "parameter_handler/StagedParameter.hpp"

using ParameterValueTypes = ::testing::Types<double, int, bool>;

template <typename ValueType_>
class ParameterTest : public ::testing::Test {
 public:
  using Parameter = parameter_handler::Parameter<ValueType_>;
  using Value = ValueType_;
};

TYPED_TEST_CASE(ParameterTest, ParameterValueTypes);

TEST(ParameterTest, setters) {  // NOLINT
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

TEST(ParameterTest, testInt) {  // NOLINT
  parameter_handler::Parameter<int> paramInt;
  paramInt.setValue(4);
  EXPECT_EQ(4, paramInt.getValue());
}

TEST(ParameterTest, testStagedInt) {  // NOLINT
  parameter_handler::StagedParameter<int> paramInt{4};
  EXPECT_EQ(4, paramInt.getValue());
  paramInt.setValue(2);
  EXPECT_EQ(4, paramInt.getValue());
  paramInt.updateValue();
  EXPECT_EQ(2, paramInt.getValue());
}