/*
 * Parameter_tests.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: Christian Gehring
 */


#include <gtest/gtest.h>
#include "parameter_handler/Parameter.hpp"




TEST (ParameterTest, constructor) {
  using namespace parameter_handler;
  ParameterDouble param(1.0, 2.0, 3.0);
  EXPECT_EQ(1.0, param.getCurrentValue());
  EXPECT_EQ(1.0, param.getDefaultValue());
  EXPECT_EQ(2.0, param.getMinValue());
  EXPECT_EQ(3.0, param.getMaxValue());
}

TEST (ParameterTest, setters) {
  using namespace parameter_handler;
  ParameterDouble param;
  param.setCurrentValue(1.0);
  EXPECT_EQ(1.0, param.getCurrentValue());

  param.setMinValue(2.0);
  EXPECT_EQ(2.0, param.getMinValue());

  param.setMaxValue(3.0);
  EXPECT_EQ(3.0, param.getMaxValue());

  param.setDefaultValue(4.0);
  EXPECT_EQ(4.0, param.getDefaultValue());

}


TEST (ParameterTest, testInt) {
  using namespace parameter_handler;

  ParameterInt paramInt;
  paramInt.setCurrentValue(4);
  EXPECT_EQ(4, paramInt.getCurrentValue());

}
