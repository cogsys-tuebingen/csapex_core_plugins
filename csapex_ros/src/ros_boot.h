#ifndef ROS_BOOT_H
#define ROS_BOOT_H

/// PROJECT
#include <csapex/core/bootstrap_plugin.h>
#include <csapex/core/core_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/view/view_fwd.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#if CLASS_LOADER_USES_HPP
#include <pluginlib/class_loader.hpp>
#else
#include <pluginlib/class_loader.h>
#endif
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

namespace csapex
{
class RosBoot : public BootstrapPlugin
{
public:
    RosBoot();
    void boot(csapex::PluginLocator* locator);

private:
    std::vector<std::string> valid_plugin_xml_files_;

    pluginlib::ClassLoader<CorePlugin> loader_core_;
    pluginlib::ClassLoader<MessageProvider> loader_msg_;
    pluginlib::ClassLoader<MessageRenderer> loader_msg_renderer_;
    pluginlib::ClassLoader<Node> loader_node_;
    pluginlib::ClassLoader<NodeAdapterBuilder> loader_node_adapter_;
    pluginlib::ClassLoader<ParameterAdapterBuilder> loader_param_adapter_;
    pluginlib::ClassLoader<DragIOHandler> loader_drag_io_;
};

}  // namespace csapex
#endif  // ROS_BOOT_H
