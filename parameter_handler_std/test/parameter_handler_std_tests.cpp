/*
 * parameter_handler_std_tests.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: gech
 */

#include <gtest/gtest.h>
#include "parameter_handler/Parameter.hpp"
#include "parameter_handler_std/ParameterHandlerStd.hpp"

TEST(ParameterTest, testDouble) {  // NOLINT
  parameter_handler::Parameter<double> param;
  param.setValue(3.0);
  EXPECT_EQ(3.0, param.getValue());

  parameter_handler::Parameter<int> paramInt;
  paramInt.setValue(4);
  EXPECT_EQ(4, paramInt.getValue());
}

TEST(ParameterHandlerStd, test) {  // NOLINT
  parameter_handler_std::ParameterHandlerStd handler;
  parameter_handler::ParameterHandlerBase* baseHandler = &handler;

  parameter_handler::Parameter<double> paramA;
  paramA.setValue(3.0);
  baseHandler->addParam("paramA", paramA);

  parameter_handler::Parameter<double> paramB;
  baseHandler->getParam("paramA", paramB);
  EXPECT_EQ(3.0, paramB.getValue());

  paramA.setValue(4.0);
  EXPECT_EQ(4.0, paramB.getValue());

  //  handler.addParam("paramA", param);
  //
  //  double valueGet = 875698758754;
  //  ASSERT_TRUE(handler.getParamValue("paramA", valueGet));
  //  ASSERT_TRUE(handler.getParamValue("paramA", valueGet));
  //  EXPECT_EQ(param.getCurrentValue(), valueGet);
  //
  //
  //
  //  //ParameterInterface<double>* paramGet;
  //  ParameterInterface<double>* paramGet;
  //  ASSERT_TRUE(handler.getParam("paramA", static_cast<ParameterInterface<double>*&>(paramGet)));
  //
  //  EXPECT_EQ(param.getCurrentValue(), paramGet->getCurrentValue());
  //
  //  ASSERT_TRUE(handler.setParamValue("paramA", 4.0));
  //  ASSERT_TRUE(handler.getParam("paramA", paramGet));
  //  EXPECT_EQ(param.getCurrentValue(), paramGet->getCurrentValue());
  //
  //  ParameterBool paramBool;
  //  ASSERT_TRUE(handler.addParam("paramB", paramBool));
}
