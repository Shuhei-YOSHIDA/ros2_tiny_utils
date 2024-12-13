/**
 * @file rqt_progressbar.cpp
 */

#include "rqt_progressbar/rqt_progressbar.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>

namespace rqt_progressbar
{

RqtProgressbar::RqtProgressbar()
  : rqt_gui_cpp::Plugin(), _widget(0)
{
  setObjectName("Rqt_test");
}

void RqtProgressbar::initPlugin(qt_gui_cpp::PluginContext& context)
{
  _widget = new QWidget();
  _ui.setupUi(_widget);

  if (context.serialNumber() > 1)
  {
    _widget->setWindowTitle(
            _widget->windowTitle()
            + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(_widget);

  // test
  std::cout << _ui.progressBar->minimum() << std::endl;
  std::cout << _ui.progressBar->maximum() << std::endl;
  _ui.progressBar->setValue(50);


  // Set event function to click on progress bar
  _ui.progressBar->installEventFilter(this);

  _clock_sub = this->node_->create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock",
          rclcpp::SensorDataQoS(),
          std::bind(&RqtProgressbar::clock_cb, this, std::placeholders::_1));
}

void RqtProgressbar::shutdownPlugin()
{

}

void RqtProgressbar::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{

}

void RqtProgressbar::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{

}

void RqtProgressbar::clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  //std::cout << "call-back" << std::endl;
}

bool RqtProgressbar::eventFilter(QObject *watched, QEvent *event)
{
  //std::cout << "called event filter" << std::endl;
  // For progress bar event
  if(watched == _ui.progressBar && event->type() == QEvent::MouseButtonPress)
  //if(watched == _ui.progressBar && event->type() == QEvent::MouseButtonDblClick)
  {
    std::cout << "pressed" << std::endl;
    return true;
  }

  // For other event, default process
  return _widget->eventFilter(watched, event);
}

} // namespace rqt_progressbar

PLUGINLIB_EXPORT_CLASS(rqt_progressbar::RqtProgressbar, rqt_gui_cpp::Plugin)
