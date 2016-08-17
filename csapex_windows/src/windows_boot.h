#ifndef ROS_BOOT_H
#define ROS_BOOT_H

/// PROJECT
#include <csapex/core/bootstrap_plugin.h>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN WindowsBoot : public BootstrapPlugin
{
public:
	WindowsBoot();
    void boot(csapex::PluginLocator *locator);

	std::string install_dir_;

private:
	template <typename PluginType>
	void get_plugin_xml_paths(std::vector<std::string>& paths);
};

}
#endif // ROS_BOOT_H
