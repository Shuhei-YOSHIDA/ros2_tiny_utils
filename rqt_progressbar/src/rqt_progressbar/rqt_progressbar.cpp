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

  _client = this->node_->create_client<rosbag2_interfaces::srv::Seek>("/rosbag2_player/seek");

  connect(_ui.startUnixLineEdit, SIGNAL(editingFinished()), this, SLOT(onLineEdit()));
  connect(_ui.endUnixLineEdit, SIGNAL(editingFinished()), this, SLOT(onLineEdit()));
  //connect(_ui.testservice, SIGNAL(pressed()), this, SLOT(onPushTestButton()));
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
  if (!s_time_str.isEmpty()) _start_time = std::stod(s_time_str.toStdString());

  QString e_time_str = instance_settings.value("end_time", "").toString();
  _ui.endUnixLineEdit->setText(e_time_str);
  if (!e_time_str.isEmpty()) _end_time = std::stod(e_time_str.toStdString());
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

      // Rate of progressbar
      double new_value = (double)min_value + ((double)click_position * (max_value - min_value))/bar_width;
      _ui.progressBar->setValue(new_value);

      std::cout << "new_value: " << new_value << std::endl;
      new_value/=(double)(max_value - min_value); // 0.0 - 1.0

      // Call rosbag-seek service
      auto request = std::make_shared<rosbag2_interfaces::srv::Seek::Request>();
      double req_time = (1.0 - new_value)*_start_time + new_value*_end_time;
      rclcpp::Time t(RCL_S_TO_NS(req_time));
      request->time = t;
      std::cout << request->time.sec << "." << request->time.nanosec << std::endl; // check
      // If request time is out of range of bagfile, rosbag2-player will CRASH!

      if (!_client->wait_for_service(10ms))
      {
        std::cout << "Service: /rosbag2_player/seek is not found" << std::endl;
        return true;
      }

      try
      {
        std::lock_guard<std::mutex> lock(_mtx);

        // test External process service call -> rosbag2_player will crash
        //std::string cmd = "ros2 service call /rosbag2_player/seek rosbag2_interfaces/srv/Seek   'time:\n  sec: 1734212500\n  nanosec: 0'";
        std::string cmd = "bash -c \"ros2 service call /rosbag2_player/seek rosbag2_interfaces/srv/Seek   'time:\n  sec: 1732412500\n  nanosec: 0' 2>&1 > /dev/null\" \n";

        // Call by `system()` -> failed
//        int ret = system(cmd.c_str());
//        std::cout << "ret: " << ret << std::endl;
//        _clock_sub->clear_on_new_message_callback();

        // Call by `popen()` -> failed
//        FILE* pipe = popen(cmd.c_str(), "r");
//        if (!pipe) return false;
//        char buffer[1024];
//        while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
//        {
//          std::cout << buffer;
//        } std::cout << std::endl;
//        pclose(pipe);

        // Call by using async_send_request -> failed
        // errmsg:[Node '/rqt_gui_cpp_node_2569069' has already been added to an executor.]
//        auto result = _client->async_send_request(request); // BAD, crash
//        // spin_until_future_complete cannot be called in this method
//        if (rclcpp::spin_until_future_complete(this->node_, result) ==
//            rclcpp::FutureReturnCode::SUCCESS)
//        {
//          std::cout << "Seek success" << std::endl;
//        }
//        else
//        {
//          std::cout << "Seek failed" << std::endl;
//        }

        // Call by usint async_send_request and callback-lambda -> failed
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
  //std::cout << "edited" << std::endl;
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

// onPushTestButton is not possible to call service?
//void RqtProgressbar::onPushTestButton()
//{
//  std::lock_guard<std::mutex> lock(_mtx);
//
//  // Failed to call service
//  //std::string cmd = "bash -c \"ros2 service call /rosbag2_player/seek rosbag2_interfaces/srv/Seek   'time:\n  sec: 1734212500\n  nanosec: 0' 2>&1 > /dev/null\" \n";
//  // Call by `system()` -> failed
//  //int ret = system(cmd.c_str());
//  //std::cout << "ret: " << ret << std::endl;
//  //_clock_sub->clear_on_new_message_callback();
//
//  // Failed to call service
//  auto request = std::make_shared<rosbag2_interfaces::srv::Seek::Request>();
//  request->time.sec = 1734212500;
//  auto result = _client->async_send_request(request,
//      [this](rclcpp::Client<rosbag2_interfaces::srv::Seek>::SharedFuture future){} );
//}

} // namespace rqt_progressbar

PLUGINLIB_EXPORT_CLASS(rqt_progressbar::RqtProgressbar, rqt_gui_cpp::Plugin)
