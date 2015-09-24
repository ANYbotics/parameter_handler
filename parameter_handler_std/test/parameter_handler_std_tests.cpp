/*
 * parameter_handler_std_tests.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: gech
 */


#include <gtest/gtest.h>
#include "parameter_handler_std/ParameterHandlerStd.hpp"
#include "parameter_handler/Parameter.hpp"


TEST(ParameterHandlerStd, test) {
  using namespace parameter_handler;
  ParameterHandlerStd handler;

  ParameterDouble param;
  param.setCurrentValue(3.0);


  handler.addParam("paramA", param);

  double valueGet = 875698758754;
  ASSERT_TRUE(handler.getParamValue("paramA", valueGet));
  ASSERT_TRUE(handler.getParamValue("paramA", valueGet));
  EXPECT_EQ(param.getCurrentValue(), valueGet);



  //ParameterInterface<double>* paramGet;
  ParameterInterface<double>* paramGet;
  ASSERT_TRUE(handler.getParam("paramA", static_cast<ParameterInterface<double>*&>(paramGet)));

  EXPECT_EQ(param.getCurrentValue(), paramGet->getCurrentValue());

  ASSERT_TRUE(handler.setParamValue("paramA", 4.0));
  ASSERT_TRUE(handler.getParam("paramA", paramGet));
  EXPECT_EQ(param.getCurrentValue(), paramGet->getCurrentValue());

  ParameterBool paramBool;
  ASSERT_TRUE(handler.addParam("paramB", paramBool));

}
