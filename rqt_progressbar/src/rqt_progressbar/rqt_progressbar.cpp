/**
 * @file rqt_progressbar.cpp
 */

#include "rqt_progressbar/rqt_progressbar.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace rqt_progressbar
{

RqtProgressbar::RqtProgressbar()
  : rqt_gui_cpp::Plugin()
{
  setObjectName("Rqt_test");
}

void RqtProgressbarinitPlugin(qt_gui_cpp::PluginContext& context)
{

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

} // namespace rqt_progressbar

PLUGINLIB_EXPORT_CLASS(rqt_progressbar::RqtProgressbar, rqt_gui_cpp::Plugin)
