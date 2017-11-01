#ifndef ROS_BOOT_H
#define ROS_BOOT_H

/// PROJECT
#include <csapex/core/bootstrap_plugin.h>
#include <csapex/core/core_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <csapex/utility/suppress_warnings_start.h>
    #include <pluginlib/class_loader.h>
#include <csapex/utility/suppress_warnings_end.h>

namespace csapex
{

class RosBoot : public BootstrapPlugin
{
public:
    RosBoot();
    void boot(csapex::PluginLocator *locator);

private:
    std::vector<std::string> valid_plugin_xml_files_;

    pluginlib::ClassLoader<CorePlugin> loader_core_;
    pluginlib::ClassLoader<MessageProvider> loader_msg_;
    pluginlib::ClassLoader<MessageRenderer> loader_msg_renderer_;
    pluginlib::ClassLoader<Node> loader_node_;
    pluginlib::ClassLoader<NodeAdapterBuilder> loader_node_adapter_;
    pluginlib::ClassLoader<DragIOHandler> loader_drag_io_;
};

}
#endif // ROS_BOOT_H
