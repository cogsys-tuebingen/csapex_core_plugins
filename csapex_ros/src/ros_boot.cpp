/// HEADER
#include "ros_boot.h"

/// PROJECT
#include <csapex/core/core_plugin.h>
#include <csapex/model/node.h>
#include <csapex/msg/message_provider.h>
#include <csapex/msg/message_renderer.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/view/designer/drag_io_handler.h>
#include <csapex/view/node/node_adapter_builder.h>
#include <csapex/view/param/param_adapter_builder.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bf3 = boost::filesystem;
#else
namespace bf3 = boost::filesystem3;
#endif

using namespace csapex;
using namespace pluginlib;

CSAPEX_REGISTER_BOOT(RosBoot)

namespace
{
std::vector<std::string> getValidPluginXMLFiles()
{
    std::vector<std::pair<std::string, std::string>> out;
    ros::package::getPlugins("csapex", "plugin", out, false);
    std::vector<std::string> vector;
    for (const auto& pair : out) {
        std::string xml = pair.second;
        if (bf3::exists(xml)) {
            vector.push_back(xml);
        }
    }
    return vector;
}
}  // namespace

RosBoot::RosBoot()
  : valid_plugin_xml_files_(getValidPluginXMLFiles())
  , loader_core_("csapex", "csapex::CorePlugin", "plugin", valid_plugin_xml_files_)
  , loader_msg_("csapex", "csapex::MessageProvider", "plugin", valid_plugin_xml_files_)
  , loader_msg_renderer_("csapex", "csapex::MessageRenderer", "plugin", valid_plugin_xml_files_)
  , loader_node_("csapex", "csapex::Node", "plugin", valid_plugin_xml_files_)
  , loader_node_adapter_("csapex", "csapex::ParameterAdapterBuilder", "plugin", valid_plugin_xml_files_)
  , loader_param_adapter_("csapex", "csapex::NodeAdapterBuilder", "plugin", valid_plugin_xml_files_)
  , loader_drag_io_("csapex", "csapex::DragIOHandler", "plugin", valid_plugin_xml_files_)
{
}

namespace
{
template <typename PluginType>
void get_plugin_xml_paths(ClassLoader<PluginType>* loader, std::vector<std::string>& paths)
{
    std::vector<std::string> files = loader->getPluginXmlPaths();
    paths.insert(paths.end(), files.begin(), files.end());
}

void loadAttributes(csapex::PluginLocator* locator, const std::string& attr_name)
{
    std::vector<std::string> attrs;
    ros::package::getPlugins("csapex", attr_name, attrs);
    locator->setPluginPaths(attr_name, attrs);
}
}  // namespace

void RosBoot::boot(csapex::PluginLocator* locator)
{
    locator->registerLocator<CorePlugin>(std::bind(&get_plugin_xml_paths<CorePlugin>, &loader_core_, std::placeholders::_1));
    locator->registerLocator<MessageProvider>(std::bind(&get_plugin_xml_paths<MessageProvider>, &loader_msg_, std::placeholders::_1));
    locator->registerLocator<MessageRenderer>(std::bind(&get_plugin_xml_paths<MessageRenderer>, &loader_msg_renderer_, std::placeholders::_1));
    locator->registerLocator<NodeAdapterBuilder>(std::bind(&get_plugin_xml_paths<NodeAdapterBuilder>, &loader_node_adapter_, std::placeholders::_1));
    locator->registerLocator<ParameterAdapterBuilder>(std::bind(&get_plugin_xml_paths<ParameterAdapterBuilder>, &loader_param_adapter_, std::placeholders::_1));
    locator->registerLocator<Node>(std::bind(&get_plugin_xml_paths<Node>, &loader_node_, std::placeholders::_1));
    locator->registerLocator<DragIOHandler>(std::bind(&get_plugin_xml_paths<DragIOHandler>, &loader_drag_io_, std::placeholders::_1));

    loadAttributes(locator, "snippets");
    loadAttributes(locator, "tutorials");
    loadAttributes(locator, "regression_tests");
}
