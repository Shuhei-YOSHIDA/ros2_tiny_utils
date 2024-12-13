/**
 * @file rqt_progressbar.cpp
 */

#include "rqt_progressbar/rqt_progressbar.hpp"
#include <cstdio>
#include <stdlib.h>
#include <exception>
#include <pluginlib/class_list_macros.hpp>
#include <qvalidator.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/qos.hpp>
#include <QMouseEvent>
#include <QDoubleValidator>

namespace rqt_progressbar
{

using namespace std::chrono_literals;

std::string convertUnixTimeToLocalTimeWithTimeZone(double unixTime)
{
  auto seconds = static_cast<std::time_t>(unixTime);
  auto nanoseconds = static_cast<long>((unixTime - seconds) * 1e9);

  std::chrono::system_clock::time_point timePoint =
    std::chrono::system_clock::from_time_t(seconds) +
    std::chrono::nanoseconds(nanoseconds);

  std::time_t localTime = std::chrono::system_clock::to_time_t(timePoint);
  std::tm* localTm = std::localtime(&localTime);

  std::ostringstream oss;

  oss << std::put_time(localTm, "%Y-%m-%d %H:%M:%S");

  oss << "." << std::setw(9) << std::setfill('0') << nanoseconds;

  char* tz = std::getenv("TZ");
  if (tz)
  {
    oss << " (" << tz << ")";
  }
  else
  {
    oss << " (";
    oss << std::put_time(localTm, "%Z") << ")";
  }

  return oss.str();
}

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

  _client = this->node_->create_client<rosbag2_interfaces::srv::Seek>("/rosbag2_player/seek");

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
  (void)plugin_settings;

  instance_settings.setValue("start_time", _ui.startUnixLineEdit->text());
  instance_settings.setValue("end_time", _ui.endUnixLineEdit->text());
  instance_settings.setValue("is_valid", _is_valid);
}

void RqtProgressbar::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  (void)plugin_settings;
  _is_valid = instance_settings.value("is_valid", false).toBool();

  QString s_time_str = instance_settings.value("start_time", "").toString();
  _ui.startUnixLineEdit->setText(s_time_str);
  if (!s_time_str.isEmpty())
  {
    _start_time = std::stod(s_time_str.toStdString());
    auto start_local_time = convertUnixTimeToLocalTimeWithTimeZone(_start_time);
    _ui.slLineEdit->setText(start_local_time.c_str());
  }

  QString e_time_str = instance_settings.value("end_time", "").toString();
  _ui.endUnixLineEdit->setText(e_time_str);
  if (!e_time_str.isEmpty())
  {
    _end_time = std::stod(e_time_str.toStdString());
    auto end_local_time = convertUnixTimeToLocalTimeWithTimeZone(_end_time);
    _ui.elLineEdit->setText(end_local_time.c_str());
  }
}

void RqtProgressbar::clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_mtx);
  if (!_is_valid) return;
  // Convert clock to percentage
  rclcpp::Time time(msg->clock);

  // Percentage of progressbar
  double v = (time.seconds() - _start_time)/(_end_time - _start_time)*_ui.progressBar->maximum();
  _ui.progressBar->setValue(v);

  auto current_local_time = convertUnixTimeToLocalTimeWithTimeZone(time.seconds());
  _ui.clLineEdit->setText(current_local_time.c_str());
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
      int min_value = _ui.progressBar->minimum(); // 0
      int max_value = _ui.progressBar->maximum(); // 100

      // Rate of progressbar
      double clicked_value = (double)min_value + ((double)click_position * (max_value - min_value))/bar_width;
      _ui.progressBar->setValue(clicked_value);

      clicked_value/=(double)(max_value - min_value); // 0.0 - 1.0

      // Call rosbag-seek service
      auto request = std::make_shared<rosbag2_interfaces::srv::Seek::Request>();
      double req_time = (1.0 - clicked_value)*_start_time + clicked_value*_end_time;
      rclcpp::Time t(RCL_S_TO_NS(req_time));
      request->time = t;
      //std::cout << request->time.sec << "." << request->time.nanosec << std::endl; // check
      //@NOTE If request time is out of range of bagfile, rosbag2-player will CRASH!

      if (!_client->wait_for_service(10ms))
      {
        RCLCPP_ERROR(this->node_->get_logger(), "Service: /rosbag2_player/seek is not found");
        return true;
      }

      try
      {
        std::lock_guard<std::mutex> lock(_mtx);

        // Call by `system()`
        //std::string cmd = "bash -c \"ros2 service call /rosbag2_player/seek rosbag2_interfaces/srv/Seek   'time:\n  sec: 1732412500\n  nanosec: 0' 2>&1 > /dev/null\" \n";
        //int ret = system(cmd.c_str());
        //std::cout << "ret: " << ret << std::endl;
        //_clock_sub->clear_on_new_message_callback();

        // Call by using async_send_request and callback-lambda
        // Can't use method of spin.. here
        auto result = _client->async_send_request(request,
            [this](rclcpp::Client<rosbag2_interfaces::srv::Seek>::SharedFuture future){} );
      }
      catch(std::exception& e)
      {
        std::cout << "FAILED: " << e.what() << std::endl;
        return false;
      }
    }

    return true;
  }

  // For other event, default process
  return _widget->eventFilter(watched, event);
}

void RqtProgressbar::onLineEdit()
{
  if (_ui.startUnixLineEdit->text().isEmpty() || _ui.endUnixLineEdit->text().isEmpty())
  {
    _is_valid = false;
    return;
  }

  _start_time = _ui.startUnixLineEdit->text().toDouble();
  _end_time = _ui.endUnixLineEdit->text().toDouble();

  auto start_time_str = convertUnixTimeToLocalTimeWithTimeZone(_start_time);
  auto end_time_str = convertUnixTimeToLocalTimeWithTimeZone(_end_time);

  _ui.slLineEdit->setText(start_time_str.c_str());
  _ui.elLineEdit->setText(end_time_str.c_str());

  _is_valid = true;
}

} // namespace rqt_progressbar

PLUGINLIB_EXPORT_CLASS(rqt_progressbar::RqtProgressbar, rqt_gui_cpp::Plugin)
