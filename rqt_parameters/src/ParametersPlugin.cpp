/*
 * ParametersPlugin.cpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */


#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>
#include <QScrollArea>
#include <QScrollBar>
#include <rqt_parameters/ParametersPlugin.hpp>
#include <ros/package.h>

#include <parameter_handler_msgs/GetParameter.h>
#include <parameter_handler_msgs/SetParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>


template<typename T>
T mapInRange (T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline double linearlyInterpolate(double v1, double v2, double t1, double t2, double t){
  if (v1 == v2)
    return v2;
  return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}

bool compareNoCase( const std::string& s1, const std::string& s2 ) {
    return strcasecmp( s1.c_str(), s2.c_str() ) <= 0;
}

struct size_less
{
    template<class T> bool operator()(T const &a, T const &b) const
    { return a.size() < b.size(); }
};

static size_t getMaxParamNameWidth(std::vector<std::string> const &lines)
{
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

    // ROS services
    getParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetParameter>("/locomotion_controller/get_parameter");
    setParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::SetParameter>("/locomotion_controller/set_parameter");
    getParameterListClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetParameterList>("/locomotion_controller/get_parameter_list");
 }



void ParametersPlugin::changeAll() {
  for (auto& param : doubleParams_) {
    param->pushButtonChangeParamPressed();
  }
}

void ParametersPlugin::refreshAll() {

  parameter_handler_msgs::GetParameterList::Request req;
  parameter_handler_msgs::GetParameterList::Response res;


  if (getParameterListClient_.call(req,res)) {
    parameters_ = res.parameters;

    // sort alphabetically
    std::sort(parameters_.begin(), parameters_.end(), compareNoCase );
    emit parametersChanged();
  }
  else {
    ROS_WARN("Could not get parameter list!");
  }
}

void ParametersPlugin::shutdownPlugin() {
  getParameterListClient_.shutdown();
  getParameterClient_.shutdown();
  setParameterClient_.shutdown();
}

void ParametersPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  plugin_settings.setValue("lineEditFilter", ui_.lineEditFilter->displayText());
}


void ParametersPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  ui_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
}

void ParametersPlugin::drawParamList() {

  doubleParams_.clear();

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

  const size_t maxParamNameWidth = getMaxParamNameWidth(parameters_);

  // Create a line for each filtered parameter
  std::string filter = ui_.lineEditFilter->text().toStdString();
  for (auto& name : parameters_) {
    std::size_t found = name.find(filter);
    if (found!=std::string::npos) {
      doubleParams_.push_back(std::shared_ptr<DoubleParameter>(new DoubleParameter(name, widget_, paramsGrid_, &getParameterClient_, &setParameterClient_, maxParamNameWidth)));
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


/*bool hasConfiguration() const
{
  return true;
}


void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/


PLUGINLIB_DECLARE_CLASS(rqt_parameter_handler, ParametersPlugin, ParametersPlugin, rqt_gui_cpp::Plugin)


