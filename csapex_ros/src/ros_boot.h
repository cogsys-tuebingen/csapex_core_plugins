#ifndef ROS_BOOT_H
#define ROS_BOOT_H

/// PROJECT
#include <csapex/core/bootstrap_plugin.h>

/// SYSTEM
#include <pluginlib/class_loader.h>

namespace csapex
{

class RosBoot : public BootstrapPlugin
{
public:
    RosBoot();
    void boot(csapex::PluginLocator *locator);

private:
    pluginlib::ClassLoader<CorePlugin> loader_core_;
    pluginlib::ClassLoader<MessageProvider> loader_msg_;
    pluginlib::ClassLoader<Node> loader_node_;
    pluginlib::ClassLoader<NodeAdapterBuilder> loader_node_adapter_;
};

}
#endif // ROS_BOOT_H
