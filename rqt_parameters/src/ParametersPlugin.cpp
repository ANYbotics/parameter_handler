/*
 * ParametersPlugin.cpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

// rqt_parameters
#include <rqt_parameters/ParametersPlugin.hpp>
#include <rqt_parameters/ParameterFloat64Matrix.hpp>
#include <QStringList>
#include <QGridLayout>
#include <QScrollArea>
#include <QScrollBar>
#include <QComboBox>

// Rqt
#include <pluginlib/class_list_macros.h>

// ros
#include <ros/package.h>
#include <rqt_parameters/ParameterInt32Matrix.hpp>

namespace rqt_parameters {

static bool compareNoCase( const std::pair<std::string, bool> & s1, const std::pair<std::string, bool> & s2 ) {
    return strcasecmp( s1.first.c_str(), s2.first.c_str() ) <= 0;
}

struct size_less {
    template<class T> bool operator()(T const &a, T const &b) const
    { return a.first.size() < b.first.size(); }
};

static size_t getMaxParamNameWidth(std::vector<std::pair<std::string, bool>> const &lines) {
  //use QFontMetrics this way;
  QFont font("", 0);
  QFontMetrics fm(font);

  auto it = std::max_element(lines.begin(), lines.end(), size_less());
  QString text = QString::fromStdString(it->first);
  return fm.width(text);
}

ParametersPlugin::ParametersPlugin() :
    rqt_gui_cpp::Plugin(),
    widget_(0),
    paramsGrid_(),
    paramsWidget_(0),
    paramsScrollHelperWidget_(0),
    paramsScrollLayout_(0)
{
  // Constructor is called first before initPlugin function, needless to say.
  // give QObjects reasonable names
  setObjectName("ParametersPlugin");
}

void ParametersPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
    // access standalone command line arguments
    QStringList argv = context.argv();

    // create the main widget, set it up and add it to the user interface
    widget_ = new QWidget();
    ui_.setupUi(widget_);
    context.addWidget(widget_);

    /******************************
     * Connect ui forms to actions *
     ******************************/
    connect(ui_.pushButtonRefreshAll, SIGNAL(pressed()), this, SLOT(refreshAll()));
    connect(ui_.lineEditFilter, SIGNAL(returnPressed()), this ,SLOT(refreshAll()));
    connect(ui_.pushButtonChangeAll, SIGNAL(pressed()), this, SLOT(changeAll()));
    connect(this, SIGNAL(parametersChanged()), this, SLOT(drawParamList()));
    connect(ui_.namespaceComboBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(setNamespace(const QString&)));
    connect(ui_.namespaceComboBox->lineEdit(), SIGNAL(returnPressed()), this, SLOT(addNamespace()));

    /******************************/

    getIntegralParameterServiceName_= "/get_integral_parameter";
    getNodeHandle().getParam("get_integral_parameter_service", getIntegralParameterServiceName_);

    setIntegralParameterServiceName_  = "/set_integral_parameter";
    getNodeHandle().getParam("set_integral_parameter_service", setIntegralParameterServiceName_);

    getFloatingPointParameterServiceName_ = "/get_floating_point_parameter";
    getNodeHandle().getParam("get_floating_point_parameter_service", getFloatingPointParameterServiceName_);

    setFloatingPointParameterServiceName_ = "/set_floating_point_parameter";
    getNodeHandle().getParam("set_floating_point_parameter_service", setFloatingPointParameterServiceName_);

    getParameterListServiceName_ = "/get_parameter_list";
    getNodeHandle().getParam("get_parameter_list_service", getParameterListServiceName_);

    setNamespace(QString(""));
}

void ParametersPlugin::shutdownServices()
{
  getParameterListClient_.shutdown();
  getIntegralParameterClient_.shutdown();
  setIntegralParameterClient_.shutdown();
  getFloatingPointParameterClient_.shutdown();
  setFloatingPointParameterClient_.shutdown();
}

bool ParametersPlugin::checkNamespace(const QString & text) {
  return (ros::service::exists(text.toStdString() + getParameterListServiceName_,false) &&
          ros::service::exists(text.toStdString() + getIntegralParameterServiceName_,false) &&
          ros::service::exists(text.toStdString() + setIntegralParameterServiceName_,false) &&
          ros::service::exists(text.toStdString() + getFloatingPointParameterServiceName_,false) &&
          ros::service::exists(text.toStdString() + setFloatingPointParameterServiceName_,false) );
}

void ParametersPlugin::addNamespace() {
  QString text = ui_.namespaceComboBox->lineEdit()->text();

  if(text == QString("clear")) {
    ui_.namespaceComboBox->clear();
    ui_.namespaceComboBox->lineEdit()->clear();
    return;
  }

  if(!checkNamespace(text)) {
    ui_.namespaceComboBox->removeItem(ui_.namespaceComboBox->findText(text));
  }

}


void ParametersPlugin::setNamespace(const QString & text) {
  shutdownServices();
  if(checkNamespace(text))
  {
    getParameterListClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetParameterList>(text.toStdString() + getParameterListServiceName_);
    getIntegralParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetIntegralParameter>(text.toStdString() + getIntegralParameterServiceName_);
    setIntegralParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::SetIntegralParameter>(text.toStdString() + setIntegralParameterServiceName_);
    getFloatingPointParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetFloatingPointParameter>(text.toStdString() + getFloatingPointParameterServiceName_);
    setFloatingPointParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::SetFloatingPointParameter>(text.toStdString() + setFloatingPointParameterServiceName_);
    refreshAll();
  }
}

void ParametersPlugin::changeAll() {
  for (auto& param : params_) {
    param->pushButtonChangeParamPressed();
  }
}

void ParametersPlugin::refreshAll() {

  parameter_handler_msgs::GetParameterList::Request req;
  parameter_handler_msgs::GetParameterList::Response res;


  if (getParameterListClient_.call(req,res)) {
    // Update parameter names
    parameterInfos_.clear();

    if(res.parameters.size() == res.isIntegral.size()) {
      for(unsigned int i = 0; i < res.parameters.size(); i++)
      {
        parameterInfos_.push_back(std::pair<std::string, bool>(res.parameters.at(i), res.isIntegral.at(i)));
      }

      // Sort names alphabetically.
      std::sort(parameterInfos_.begin(), parameterInfos_.end(), compareNoCase );

      // Update GUI
      emit parametersChanged();
    }
    else {
      ROS_WARN("Parameter list is not consistent!");
    }
  }
  else {
    ROS_WARN("Could not get parameter list!");
  }
}

void ParametersPlugin::shutdownPlugin() {
  getParameterListClient_.shutdown();
  getIntegralParameterClient_.shutdown();
  setIntegralParameterClient_.shutdown();
  getFloatingPointParameterClient_.shutdown();
  setFloatingPointParameterClient_.shutdown();
}

void ParametersPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  plugin_settings.setValue("lineEditFilter", ui_.lineEditFilter->displayText());
  plugin_settings.setValue("nrNamespaces", ui_.namespaceComboBox->count());
  plugin_settings.setValue("currentNamespace", ui_.namespaceComboBox->currentIndex());

  for(int i = 0; i<ui_.namespaceComboBox->count(); ++i) {
    plugin_settings.setValue(QString::fromStdString("ns" + std::to_string(i)), ui_.namespaceComboBox->itemText(i));
  }

}

void ParametersPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  ui_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
  for(int i = 0; i<plugin_settings.value("nrNamespaces").toInt(); ++i) {
    ui_.namespaceComboBox->insertItem(i, plugin_settings.value(QString::fromStdString("ns" + std::to_string(i))).toString());
  }
  int currIdx = plugin_settings.value("currentNamespace").toInt();
  ui_.namespaceComboBox->setCurrentIndex(currIdx);
  setNamespace(ui_.namespaceComboBox->itemText(currIdx));
}

void ParametersPlugin::drawParamList() {

  params_.clear();

  if (paramsWidget_) {
    // delete widget
    delete paramsWidget_->layout();
    delete paramsScrollHelperWidget_->layout();
    ui_.gridLayout->removeWidget(paramsWidget_);
    delete paramsWidget_;
  }

  paramsWidget_ = new QWidget();
  paramsWidget_->setObjectName(QString::fromUtf8("paramsWidget"));
  ui_.gridLayout->addWidget(paramsWidget_, 10, 1, 1, 1);


  paramsScrollHelperWidget_ = new QWidget(paramsWidget_);
  paramsGrid_= new QGridLayout(paramsScrollHelperWidget_);
  paramsGrid_->setSpacing(6);
  paramsGrid_->setObjectName(QString::fromUtf8("paramsGrid"));

  const size_t maxParamNameWidth = getMaxParamNameWidth(parameterInfos_);

  // Create a line for each filtered parameter
  std::string filter = ui_.lineEditFilter->text().toStdString();
  for (auto& info : parameterInfos_) {
    std::size_t found = info.first.find(filter);
    if (found!=std::string::npos) {
      if(info.second) {
        params_.push_back(std::shared_ptr<ParameterBase>(
            new ParameterInt32Matrix(info.first, paramsGrid_, &getIntegralParameterClient_, &setIntegralParameterClient_)));
      } else {
        params_.push_back(std::shared_ptr<ParameterBase>(
            new ParameterFloat64Matrix(info.first, paramsGrid_, &getFloatingPointParameterClient_, &setFloatingPointParameterClient_)));
      }
      params_.back()->setupGUI(widget_, maxParamNameWidth);
    }
  }
  // This needs to be done after everthing is setup.
  paramsScrollHelperWidget_->setLayout(paramsGrid_);

  // Put it into a scroll area
  QScrollArea* paramsScrollArea = new QScrollArea();
  paramsScrollArea->setWidget(paramsScrollHelperWidget_);

  // Make the scroll step the same width as the fixed widgets in the grid
  paramsScrollArea->horizontalScrollBar()->setSingleStep(paramsScrollHelperWidget_->width() / 24);

  paramsScrollLayout_ = new QVBoxLayout(paramsWidget_);
  paramsScrollLayout_->addWidget(paramsScrollArea);

  paramsWidget_->setLayout(paramsScrollLayout_);
  paramsScrollHelperWidget_->setLayout(paramsGrid_);

}

PLUGINLIB_DECLARE_CLASS(rqt_parameter_handler, ParametersPlugin, ParametersPlugin, rqt_gui_cpp::Plugin)

}
