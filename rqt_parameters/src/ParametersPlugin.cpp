/*
 * ParametersPlugin.cpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

// rqt_parameters
#include <rqt_parameters/ParametersPlugin.hpp>
#include <rqt_parameters/ParameterFloat64Matrix.hpp>

// Qt
#include <QStringList>
#include <QGridLayout>
#include <QScrollArea>
#include <QScrollBar>

// Rqt
#include <pluginlib/class_list_macros.h>

// ros
#include <ros/package.h>

namespace rqt_parameters {

static bool compareNoCase( const std::string& s1, const std::string& s2 ) {
    return strcasecmp( s1.c_str(), s2.c_str() ) <= 0;
}

struct size_less {
    template<class T> bool operator()(T const &a, T const &b) const
    { return a.size() < b.size(); }
};

static size_t getMaxParamNameWidth(std::vector<std::string> const &lines) {
  //use QFontMetrics this way;
  QFont font("", 0);
  QFontMetrics fm(font);

  auto it = std::max_element(lines.begin(), lines.end(), size_less());
  QString text = QString::fromStdString(*it);
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
    /******************************/

    std::string getIntegralParameterServiceName{"/rocoma_example/get_integral_parameter"};
    getNodeHandle().getParam("get_integral_parameter_service", getIntegralParameterServiceName);

    std::string setIntegralParameterServiceName{"/rocoma_example/set_integral_parameter"};
    getNodeHandle().getParam("set_integral_parameter_service", setIntegralParameterServiceName);

    std::string getFloatingPointParameterServiceName{"/rocoma_example/get_floating_point_parameter"};
    getNodeHandle().getParam("get_floating_point_parameter_service", getFloatingPointParameterServiceName);

    std::string setFloatingPointParameterServiceName{"/rocoma_example/set_floating_point_parameter"};
    getNodeHandle().getParam("set_floating_point_parameter_service", setFloatingPointParameterServiceName);

    std::string getParameterListServiceName{"/rocoma_example/get_parameter_list"};
    getNodeHandle().getParam("get_parameter_list_service", getParameterListServiceName);

    // ROS services
    getParameterListClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetParameterList>(getParameterListServiceName);
    getIntegralParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetIntegralParameter>(getIntegralParameterServiceName);
    setIntegralParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::SetIntegralParameter>(setIntegralParameterServiceName);
    getFloatingPointParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetFloatingPointParameter>(getFloatingPointParameterServiceName);
    setFloatingPointParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::SetFloatingPointParameter>(setFloatingPointParameterServiceName);
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
    parameterNames_ = res.parameters;

    // Sort names alphabetically.
    std::sort(parameterNames_.begin(), parameterNames_.end(), compareNoCase );

    // Update GUI
    emit parametersChanged();
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
}

void ParametersPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  ui_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
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

  const size_t maxParamNameWidth = getMaxParamNameWidth(parameterNames_);

  // Create a line for each filtered parameter
  std::string filter = ui_.lineEditFilter->text().toStdString();
  for (auto& name : parameterNames_) {
    std::size_t found = name.find(filter);
    if (found!=std::string::npos) {
      params_.push_back(std::shared_ptr<ParameterBase>(
          new ParameterFloat64Matrix(name, paramsGrid_, &getFloatingPointParameterClient_, &setFloatingPointParameterClient_)));
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

