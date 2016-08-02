/*
 * ParametersPlugin.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
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

static size_t max_line_length(std::vector<std::string> const &lines)
{
  //use QFontMetrics this way;
  QFont font("", 0);
  QFontMetrics fm(font);

  auto it = std::max_element(lines.begin(), lines.end(), size_less());
  QString text = QString::fromStdString(*it);
  return fm.width(text);;
}

ParametersPlugin::ParametersPlugin() :
    rqt_gui_cpp::Plugin(),
    widget_(0),
    wScroll_(0),
    client_(0),
    layout_(0)
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
    connect(this, SIGNAL(parametersChanged()), this, SLOT(drawParamList()));
    /******************************/



    getParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetParameter>("/locomotion_controller/get_parameter");
    setParameterClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::SetParameter>("/locomotion_controller/set_parameter");
    getParameterListClient_ = getNodeHandle().serviceClient<parameter_handler_msgs::GetParameterList>("/locomotion_controller/get_parameter_list");
 }



void ParametersPlugin::pushButtonChangeAllPressed() {
  for (auto& param : doubleParams_) {
    param->pushButtonChangeParamPressed();
  }
}

void ParametersPlugin::pushButtonRefreshAllPressed() {

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

  if (wScroll_) {
    // delete widget
    delete wScroll_->layout();
    delete client_->layout();
    ui_.gridLayout->removeWidget(wScroll_);
    delete wScroll_;
  }

  wScroll_ = new QWidget();
  wScroll_->setObjectName(QString::fromUtf8("wScroll"));

  ui_.gridLayout->addWidget(wScroll_, 10, 1, 1, 1);


  client_ = new QWidget(wScroll_);
  newGridParams_= new QGridLayout(client_);
  newGridParams_->setSpacing(6);
  newGridParams_->setObjectName(QString::fromUtf8("newGridParams"));

  size_t maxParamNameSize = max_line_length(parameters_);

  std::string filter = ui_.lineEditFilter->text().toStdString();
  for (auto& name : parameters_) {
    if (name.compare(0, filter.length(), filter) == 0) {
      doubleParams_.push_back(std::shared_ptr<DoubleParameter>(new DoubleParameter(name, widget_, newGridParams_, &getParameterClient_, &setParameterClient_, maxParamNameSize)));
    }
  }
  // This needs to be done after everthing is setup.
  client_->setLayout(newGridParams_);

  // Put it into a scroll area
  QScrollArea* area = new QScrollArea();
  area->setWidget(client_);

  // Make the scroll step the same width as the fixed widgets in the grid
  area->horizontalScrollBar()->setSingleStep(client_->width() / 24);

  layout_ = new QVBoxLayout(wScroll_);
  layout_->addWidget(area);

  wScroll_->setLayout(layout_);
  client_->setLayout(newGridParams_);

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


