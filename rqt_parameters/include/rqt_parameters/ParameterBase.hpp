/*
 * ParameterBase.hpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

#pragma once

// rqt_parameters
#include "rqt_parameters/MatrixSpinBox.hpp"

// Qt
#include <QWidget>
#include <QStringList>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QApplication>

// ros
#include <ros/ros.h>

//! This class draws and handles a double parameter.
class ParameterBase: public QObject {
  Q_OBJECT
 public:
  ParameterBase(const std::string& name,
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

    std::string lineEditName = std::string{"lineEdit"} + name;
    labelParamName = new QLabel(widget);
    labelParamName->setObjectName(QString::fromStdString(lineEditName));
    //    lineEditParamName->setMinimumSize(QSize(100, 0));
    labelParamName->setText(QString::fromStdString(name));
    labelParamName->setFixedWidth(maxParamNameWidth-10);

    std::string spinBoxName = std::string{"spinBox"} + name;
    matrixSpinBoxParamValue = new MatrixSpinBox(widget);
    matrixSpinBoxParamValue->setObjectName(QString::fromStdString(spinBoxName));
    matrixSpinBoxParamValue->setMinimumSize(QSize(200, 0));
    //    matrixSpinBoxParamValue->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    //    matrixSpinBoxParamValue->setMaximum(1e+08);
    //    matrixSpinBoxParamValue->setSingleStep(10);
    //    matrixSpinBoxParamValue->setFixedSize(spinBoxParamValue->width(), spinBoxParamValue->height());

    std::string pushButtonName = std::string{"pushButtonChange"} + name;
    pushButtonChangeParam = new QPushButton(widget);
    pushButtonChangeParam->setObjectName(QString::fromStdString(pushButtonName));
    pushButtonChangeParam->setMaximumSize(QSize(60, 16777215));
    pushButtonChangeParam->setText(QApplication::translate("ParametersHandler", "change", 0, QApplication::UnicodeUTF8));
    //    pushButtonChangeParam->setFixedSize(pushButtonChangeParam->width(), pushButtonChangeParam->height());

    grid->addWidget(labelParamNumber,         iRow, 0, 1, 1, Qt::AlignTop);
    grid->addWidget(labelParamName,           iRow, 1, 1, 1, Qt::AlignTop);
    grid->addWidget(matrixSpinBoxParamValue,  iRow, 2, 1, 1, Qt::AlignTop);
    grid->addWidget(pushButtonChangeParam,    iRow, 3, 1, 1, Qt::AlignTop);

    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(pushButtonChangeParamPressed()));

//    refreshParam();
  }

  virtual ~ParameterBase() {
    disconnect(pushButtonChangeParam, SIGNAL(pressed()), 0, 0);

    grid_->removeWidget(labelParamNumber);
    grid_->removeWidget(labelParamName);
    grid_->removeWidget(matrixSpinBoxParamValue);
    grid_->removeWidget(pushButtonChangeParam);

    delete labelParamNumber;
    delete labelParamName;
    delete matrixSpinBoxParamValue;
    delete pushButtonChangeParam;
  };

 public slots:
  virtual void pushButtonChangeParamPressed() = 0;
  virtual void refreshParam() = 0;

 protected:
  std::string name_;
  ros::ServiceClient* getParameterClient_;
  ros::ServiceClient* setParameterClient_;
  QGridLayout* grid_;

 public:
  QLabel* labelParamNumber;
  QLabel* labelParamName;
  MatrixSpinBox* matrixSpinBoxParamValue;
  QPushButton* pushButtonChangeParam;
};
