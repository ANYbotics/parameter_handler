/*
 * ParametersPlugin.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
 */


#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>
#include <rqt_parameters/ParametersPlugin.hpp>
#include <ros/package.h>

#include <parameter_handler_msgs/GetParameter.h>
#include <parameter_handler_msgs/SetParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>


#define SPEED_MIN 0.1
#define SPEED_MAX 4.0
#define SPEED_RESET 1.0

#define TIME_MIN 0
#define TIME_MAX 1
#define TIME_RESET 0.0

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

ParametersPlugin::ParametersPlugin() :
    rqt_gui_cpp::Plugin(),
    widget_(0)
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
    connect(ui_.pushButtonRefreshAll, SIGNAL(pressed()), this, SLOT(pushButtonRefreshAllPressed()));
    connect(ui_.pushButtonChangeAll, SIGNAL(pressed()), this, SLOT(pushButtonChangeAllPressed()));

    /******************************/


    getParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetParameter>("/locomotion_controller/get_parameter");
    setParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::SetParameter>("/locomotion_controller/set_parameter");
    getParameterListClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetParameterList>("/locomotion_controller/get_parameter_list");



    doubleParams_.push_back(std::shared_ptr<DoubleParameter>(new DoubleParameter("test", widget_, ui_.gridParams, &getParameterClient_, &setParameterClient_)));
    doubleParams_.push_back(std::shared_ptr<DoubleParameter>(new DoubleParameter("test2", widget_, ui_.gridParams, &getParameterClient_, &setParameterClient_)));
//    int iRow = 3;
//    ui_.gridParams->addWidget(doubleParams_.back()->lineEditParamName,     iRow, 0, 1, 1);
//    ui_.gridParams->addWidget(doubleParams_.back()->spinBoxParamValue,     iRow, 1, 1, 1);
//    ui_.gridParams->addWidget(doubleParams_.back()->pushButtonChangeParam, iRow, 2, 1, 1);

 }



void ParametersPlugin::pushButtonChangeAllPressed() {
  for (auto& param : doubleParams_) {
    param->pushButtonChangeParamPressed();
  }
}

void ParametersPlugin::pushButtonRefreshAllPressed() {

  parameter_handler_msgs::GetParameterList::Request req;
  parameter_handler_msgs::GetParameterList::Response res;

  doubleParams_.clear();


//    ui_.comboBoxAvailableParameters->clear();

    if (getParameterListClient_.call(req,res)) {
      // sort alphabetically
      std::sort(res.parameters.begin(), res.parameters.end(), compareNoCase );
      for (auto& name : res.parameters) {
        doubleParams_.push_back(std::shared_ptr<DoubleParameter>(new DoubleParameter(name, widget_, ui_.gridParams, &getParameterClient_, &setParameterClient_)));
      }
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
//  plugin_settings.setValue("lineEditParamName", ui_.lineEditParamName->displayText());
//  plugin_settings.setValue("spinBoxParamValue", ui_.spinBoxParamValue->value());

}


void ParametersPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {

  ui_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
//  ui_.lineEditParamName->setText(plugin_settings.value("lineEditParamName").toString());
//  ui_.spinBoxParamValue->setValue(plugin_settings.value("spinBoxParamValue").toDouble());

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


