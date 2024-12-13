/**
 * @file rqt_progressbar.cpp
 */

#include "rqt_progressbar/rqt_progressbar.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <qvalidator.h>
#include <rclcpp/qos.hpp>
#include <QMouseEvent>
#include <QDoubleValidator>

namespace rqt_progressbar
{

RqtProgressbar::RqtProgressbar()
  : rqt_gui_cpp::Plugin(), _widget(0)
{
  setObjectName("RqtProgressbarApp");
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
  //std::cout << _ui.progressBar->minimum() << std::endl;
  //std::cout << _ui.progressBar->maximum() << std::endl;

  _ui.progressBar->setValue(0);

  // Set event function to click on progress bar
  _ui.progressBar->installEventFilter(this);

  // Validator for LineEdit
  QDoubleValidator *validator = new QDoubleValidator(0.0, 2147483647.0, 9); // 2023 problem
  validator->setNotation(QDoubleValidator::StandardNotation);
  _ui.startUnixLineEdit->setValidator(validator);
  _ui.endUnixLineEdit->setValidator(validator);

  _clock_sub = this->node_->create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock",
          rclcpp::SensorDataQoS(),
          std::bind(&RqtProgressbar::clock_cb, this, std::placeholders::_1));

  connect(_ui.startUnixLineEdit, SIGNAL(editingFinished()), this, SLOT(onLineEdit()));
  connect(_ui.endUnixLineEdit, SIGNAL(editingFinished()), this, SLOT(onLineEdit()));
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
  if (!_is_valid) return;
  // Convert clock to percentage
  rclcpp::Time time(msg->clock);

  double v = (time.seconds() - _start_time)/(_end_time - _start_time)*_ui.progressBar->maximum();
  _ui.progressBar->setValue(v);
}

bool RqtProgressbar::eventFilter(QObject *watched, QEvent *event)
{
  // For progress bar event
  if(watched == _ui.progressBar && event->type() == QEvent::MouseButtonPress)
  {
    auto *mouse_event = static_cast<QMouseEvent *>(event);
    if (mouse_event->button() == Qt::LeftButton)
    {
      int bar_width = _ui.progressBar->width();
      int click_position = mouse_event->pos().x();
      int min_value = _ui.progressBar->minimum();
      int max_value = _ui.progressBar->maximum();

      int new_value = min_value + (click_position * (max_value - min_value))/bar_width;
      _ui.progressBar->setValue(new_value);

      // Call rosbag-seek service
    }

    return true;
  }

  // For other event, default process
  return _widget->eventFilter(watched, event);
}

void RqtProgressbar::onLineEdit()
{
  std::cout << "edited" << std::endl;
  if (_ui.startUnixLineEdit->text().isEmpty() || _ui.endUnixLineEdit->text().isEmpty())
  {
    _is_valid = false;
    return;
  }

  _start_time = _ui.startUnixLineEdit->text().toDouble();
  _end_time = _ui.endUnixLineEdit->text().toDouble();

  //std::cout << _start_time << std::endl;
  //std::cout << _end_time << std::endl;

  _is_valid = true;
}

} // namespace rqt_progressbar

PLUGINLIB_EXPORT_CLASS(rqt_progressbar::RqtProgressbar, rqt_gui_cpp::Plugin)
