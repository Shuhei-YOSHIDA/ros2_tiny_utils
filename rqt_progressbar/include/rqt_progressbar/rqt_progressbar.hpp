/**
 * @file rqt_progressbar.hpp
 * @brief Move progressbar with /clock topic
 *  and call /rosbag2_player/seek service by clicking bar
 */

#ifndef INCLUDE_RQT_PROGRESSBAR_RQT_PROGRESSBAR_HPP
#define INCLUDE_RQT_PROGRESSBAR_RQT_PROGRESSBAR_HPP

#include <qcoreevent.h>
#include <qobject.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rosbag2_interfaces/srv/seek.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>
#include <rosbag2_interfaces/srv/play_next.hpp>

#include <rqt_gui_cpp/plugin.h>
#include "ui_rqt_progressbar.h"

namespace rqt_progressbar
{

class RqtProgressbar
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  RqtProgressbar();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
  virtual void shutdownPlugin() override;
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const override;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings) override;

protected:
  Ui::RqtProgressbarWidget _ui;
  QWidget* _widget;

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr _clock_sub;
  void clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg);

  rclcpp::Client<rosbag2_interfaces::srv::Seek>::SharedPtr _client_seek;
  rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr _client_togglepaused;
  rclcpp::Client<rosbag2_interfaces::srv::PlayNext>::SharedPtr _client_playnext;

  bool eventFilter(QObject *watched, QEvent *event) override;

  bool _is_valid = false;
  double _start_time;
  double _end_time;

  std::mutex _mtx;

protected slots:
  virtual void onLineEdit();
  virtual void onToggleButtonPushed();
  virtual void onPlayNextButtonPushed();
};

} // namespace rqt_progressbar

#endif /* ifndef INCLUDE_RQT_PROGRESSBAR_RQT_PROGRESSBAR_HPP */
