/*
 * ParametersPlugin.hpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_parameters_plugin.h>
#include <QWidget>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <parameter_handler_msgs/GetParameter.h>
#include <parameter_handler_msgs/SetParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>

#include <QDoubleSpinBox>

#include <list>
#include <memory>

class DoubleParameter: public QObject {
  Q_OBJECT
 public:
  DoubleParameter(const std::string& name,
                  QWidget* widget,
                  QGridLayout* grid,
                  ros::ServiceClient* getParameterClient,
                  ros::ServiceClient* setParameterClient) {
    name_ = name;
    grid_ = grid;
    getParameterClient_ = getParameterClient;
    setParameterClient_ = setParameterClient;

    std::string lineEditName = std::string{"lineEdit"} + name;
    lineEditParamName = new QLineEdit(widget);
    lineEditParamName->setObjectName(QString::fromStdString(lineEditName));
    lineEditParamName->setMinimumSize(QSize(100, 0));
    lineEditParamName->setText(QString::fromStdString(name));

    std::string spinBoxName = std::string{"spinBox"} + name;
    spinBoxParamValue = new QDoubleSpinBox(widget);
    spinBoxParamValue->setObjectName(QString::fromStdString(spinBoxName));
    spinBoxParamValue->setMinimumSize(QSize(200, 0));
    spinBoxParamValue->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxParamValue->setMaximum(1e+08);
    spinBoxParamValue->setSingleStep(10);

    std::string pushButtonName = std::string{"pushButtonChange"} + name;
    pushButtonChangeParam = new QPushButton(widget);
    pushButtonChangeParam->setObjectName(QString::fromStdString(pushButtonName));
    pushButtonChangeParam->setMaximumSize(QSize(60, 16777215));
    pushButtonChangeParam->setText(QApplication::translate("ParametersHandler", "change", 0, QApplication::UnicodeUTF8));

    int iRow = grid->rowCount()+1;
    grid->addWidget(lineEditParamName,     iRow, 0, 1, 1);
    grid->addWidget(spinBoxParamValue,     iRow, 1, 1, 1);
    grid->addWidget(pushButtonChangeParam, iRow, 2, 1, 1);

//    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(pushButtonChangeParamPressed()));
    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(pushButtonChangeParamPressed()));

  }
  virtual ~DoubleParameter() {
//    grid_->removeWidget(this);
    delete lineEditParamName;
    delete spinBoxParamValue;
    delete pushButtonChangeParam;
  };

  public slots:

  void pushButtonChangeParamPressed() {
    ROS_INFO_STREAM("Change parameter " << name_ << " with value " << spinBoxParamValue->value());

    parameter_handler_msgs::GetParameterRequest reqGet;
    parameter_handler_msgs::GetParameterResponse resGet;

    reqGet.name = name_;
    getParameterClient_->call(reqGet, resGet);

    parameter_handler_msgs::SetParameterRequest req;
    parameter_handler_msgs::SetParameterResponse res;

    req.name = name_;
    req.value = spinBoxParamValue->value();

    if(!setParameterClient_->call(req, res)) {
      ROS_ERROR_STREAM("Could not set parameter " << name_);
    }

  }
 protected:
  std::string name_;
  ros::ServiceClient* getParameterClient_;
  ros::ServiceClient* setParameterClient_;
  QGridLayout* grid_;
 public:
  QLineEdit* lineEditParamName;
  QDoubleSpinBox* spinBoxParamValue;
  QPushButton* pushButtonChangeParam;
};

class ParametersPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
public:
  ParametersPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected:


  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
  private:
  Ui::ParametersHandler ui_;
  QWidget* widget_;

  // Parameters
  ros::ServiceClient getParameterListClient_;
  ros::ServiceClient getParameterClient_;
  ros::ServiceClient setParameterClient_;


  std::list<std::shared_ptr<DoubleParameter>> doubleParams_;

 protected slots:

  // Parameters
  void pushButtonRefreshAllPressed();
  void pushButtonChangeAllPressed();

signals:
    void stateChanged();


};


