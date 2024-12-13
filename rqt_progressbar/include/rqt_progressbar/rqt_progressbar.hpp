/**
 * @file rqt_progressbar.hpp
 */

#ifndef INCLUDE_RQT_PROGRESSBAR_RQT_PROGRESSBAR_HPP
#define INCLUDE_RQT_PROGRESSBAR_RQT_PROGRESSBAR_HPP

#include <qcoreevent.h>
#include <qobject.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

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

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

protected:
  Ui::RqtProgressbarWidget _ui;
  QWidget* _widget;

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr _clock_sub;
  void clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg);

  bool eventFilter(QObject *watched, QEvent *event) override;

};

} // namespace rqt_progressbar

#endif /* ifndef INCLUDE_RQT_PROGRESSBAR_RQT_PROGRESSBAR_HPP */
