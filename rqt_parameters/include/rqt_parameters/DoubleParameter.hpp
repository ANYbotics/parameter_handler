/*
 * DoubleParameter.hpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <QWidget>
#include <QStringList>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QApplication>

#include <ros/ros.h>

#include <parameter_handler_msgs/GetParameter.h>
#include <parameter_handler_msgs/SetParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>

class DoubleParameter: public QObject {
  Q_OBJECT
 public:
  DoubleParameter(const std::string& name,
                  QWidget* widget,
                  QGridLayout* grid,
                  ros::ServiceClient* getParameterClient,
                  ros::ServiceClient* setParameterClient,
                  size_t maxParamNameWidth) {
    name_ = name;
    grid_ = grid;
    getParameterClient_ = getParameterClient;
    setParameterClient_ = setParameterClient;
    int iRow = grid->rowCount();


    labelParamNumber = new QLabel(widget);
    labelParamNumber->setObjectName(QString::fromUtf8("labelParamNumber"));
    labelParamNumber->setText(QString::number(iRow)+QString::fromUtf8(")"));
//    labelParamNumber->setAlignment(Qt::AlignRight);

    std::string lineEditName = std::string{"lineEdit"} + name;
    labelParamName = new QLabel(widget);
    labelParamName->setObjectName(QString::fromStdString(lineEditName));
//    lineEditParamName->setMinimumSize(QSize(100, 0));
    labelParamName->setText(QString::fromStdString(name));


    labelParamName->setFixedSize(maxParamNameWidth-10, labelParamName->height());

    std::string spinBoxName = std::string{"spinBox"} + name;
    spinBoxParamValue = new QDoubleSpinBox(widget);
    spinBoxParamValue->setObjectName(QString::fromStdString(spinBoxName));
    spinBoxParamValue->setMinimumSize(QSize(200, 0));
    spinBoxParamValue->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxParamValue->setMaximum(1e+08);
    spinBoxParamValue->setSingleStep(10);
//    spinBoxParamValue->setFixedSize(spinBoxParamValue->width(), spinBoxParamValue->height());

    std::string pushButtonName = std::string{"pushButtonChange"} + name;
    pushButtonChangeParam = new QPushButton(widget);
    pushButtonChangeParam->setObjectName(QString::fromStdString(pushButtonName));
    pushButtonChangeParam->setMaximumSize(QSize(60, 16777215));
    pushButtonChangeParam->setText(QApplication::translate("ParametersHandler", "change", 0, QApplication::UnicodeUTF8));
//    pushButtonChangeParam->setFixedSize(pushButtonChangeParam->width(), pushButtonChangeParam->height());

    labelParamLimits = new QLabel(widget);
    labelParamLimits->setObjectName(QString::fromUtf8("labelParamLimits"));
    labelParamLimits->setText(QString::fromUtf8("[?, ?]"));

    grid->addWidget(labelParamNumber,      iRow, 0, 1, 1);
    grid->addWidget(labelParamName,        iRow, 1, 1, 1);
    grid->addWidget(spinBoxParamValue,     iRow, 2, 1, 1);
    grid->addWidget(pushButtonChangeParam, iRow, 3, 1, 1);
    grid->addWidget(labelParamLimits,      iRow, 4, 1, 1);

    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(pushButtonChangeParamPressed()));

    refreshParam();
  }
  virtual ~DoubleParameter() {
    disconnect(pushButtonChangeParam, SIGNAL(pressed()), 0, 0);

    grid_->removeWidget(labelParamNumber);
    grid_->removeWidget(labelParamName);
    grid_->removeWidget(labelParamLimits);
    grid_->removeWidget(spinBoxParamValue);
    grid_->removeWidget(pushButtonChangeParam);

    delete labelParamNumber;
    delete labelParamName;
    delete labelParamLimits;
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

  void refreshParam() {

    parameter_handler_msgs::GetParameterRequest req;
    parameter_handler_msgs::GetParameterResponse res;

    req.name = name_;
    if (getParameterClient_->exists()) {
      if (getParameterClient_->call(req, res)) {
        spinBoxParamValue->setMinimum(res.value_min);
        spinBoxParamValue->setMaximum(res.value_max);
        spinBoxParamValue->setValue( res.value_current );
        spinBoxParamValue->setRange(res.value_min, res.value_max);
        spinBoxParamValue->setSingleStep(std::abs(res.value_max-res.value_min)/10.0);
        labelParamLimits->setText(QString::fromUtf8("[") + QString::number(res.value_min) + QString::fromUtf8(", ") + QString::number(res.value_max)  + QString::fromUtf8("]"));
      }
      else {
        ROS_WARN_STREAM("Could not get parameter " << name_);
      }
    }


  }
 protected:
  std::string name_;
  ros::ServiceClient* getParameterClient_;
  ros::ServiceClient* setParameterClient_;
  QGridLayout* grid_;
 public:
  QLabel* labelParamNumber;
  QLabel* labelParamName;
  QLabel* labelParamLimits;
  QDoubleSpinBox* spinBoxParamValue;
  QPushButton* pushButtonChangeParam;
};