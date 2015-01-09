/// HEADER
#include "ros_boot.h"

/// PROJECT
#include <csapex/plugin/plugin_locator.h>
#include <csapex/core/core_plugin.h>
#include <csapex/model/node.h>
#include <csapex/msg/message_provider.h>
#include <csapex/msg/message_renderer.h>
#include <csapex/view/node_adapter_builder.h>

using namespace csapex;
using namespace pluginlib;

CSAPEX_REGISTER_BOOT(RosBoot)

RosBoot::RosBoot() :
    loader_core_("csapex", "csapex::CorePlugin"),
    loader_msg_("csapex", "csapex::MessageProvider"),
    loader_msg_renderer_("csapex", "csapex::MessageRenderer"),
    loader_node_("csapex", "csapex::Node"),
    loader_node_adapter_("csapex", "csapex::NodeAdapterBuilder")
{

}

namespace {
template <typename PluginType>
void get_plugin_xml_paths(ClassLoader<PluginType>* loader, std::vector<std::string>& paths)
{
    std::vector<std::string> files = loader->getPluginXmlPaths();
    paths.insert(paths.end(), files.begin(), files.end());
}
}

void RosBoot::boot(csapex::PluginLocator* locator)
{
    locator->registerLocator<CorePlugin>(boost::bind(
                                             &get_plugin_xml_paths<CorePlugin>,
                                             &loader_core_, _1));
    locator->registerLocator<MessageProvider>(boost::bind(
                                                  &get_plugin_xml_paths<MessageProvider>,
                                                  &loader_msg_, _1));
    locator->registerLocator<MessageRenderer>(boost::bind(
                                                  &get_plugin_xml_paths<MessageRenderer>,
                                                  &loader_msg_renderer_, _1));
    locator->registerLocator<NodeAdapterBuilder>(boost::bind(
                                                     &get_plugin_xml_paths<NodeAdapterBuilder>,
                                                     &loader_node_adapter_, _1));
    locator->registerLocator<Node>(boost::bind(
                                       &get_plugin_xml_paths<Node>,
                                       &loader_node_, _1));
}
