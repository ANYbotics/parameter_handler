/*
 * ParametersPlugin.hpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_parameters_plugin.h>
#include <QWidget>
#include <QDoubleSpinBox>
#include <QScrollArea>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <parameter_handler_msgs/GetParameter.h>
#include <parameter_handler_msgs/SetParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>

#include <rqt_parameters/DoubleParameter.hpp>

#include <list>
#include <memory>

class ParametersPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
public:
  ParametersPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
private:
  Ui::ParametersHandler ui_;
  QWidget* widget_;
  QGridLayout* paramsGrid_;
  QWidget* paramsWidget_;
  QWidget* paramsScrollHelperWidget_;
  QVBoxLayout* paramsScrollLayout_;

  // ROS services
  ros::ServiceClient getParameterListClient_;
  ros::ServiceClient getParameterClient_;
  ros::ServiceClient setParameterClient_;

  std::list<std::shared_ptr<DoubleParameter>> doubleParams_;
  std::vector<std::string> parameterNames_;

 protected slots:
  void refreshAll();
  void changeAll();
  void drawParamList();

signals:
  void parametersChanged();

};


