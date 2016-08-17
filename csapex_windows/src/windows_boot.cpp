/// HEADER
#include "windows_boot.h"

/// PROJECT
#include <csapex/plugin/plugin_locator.h>
#include <csapex/model/node.h>
#include <csapex/core/core_plugin.h>
#include <csapex/msg/message_provider.h>
#include <csapex/view/message_renderer.h>
#include <csapex/view/node/node_adapter_builder.h>
#include <csapex/view/designer/drag_io_handler.h>

/// SYSTEM
#include <Windows.h>
#include <iostream>
#include <malloc.h>
#include <stdio.h>
#include <boost/filesystem.hpp>

using namespace csapex;
namespace bfs = boost::filesystem;

CSAPEX_REGISTER_BOOT(csapex::WindowsBoot)


WindowsBoot::WindowsBoot()
{
	// find install dir
	char ownPth[MAX_PATH];
	HMODULE hModule = GetModuleHandle(NULL);
	if (hModule != NULL)
	{
		GetModuleFileNameA(hModule, ownPth, (sizeof(ownPth)));

	} else {
		throw std::runtime_error("cannot get path of own executable");
	}

	std::string exe_dir(ownPth);
	std::string bin_dir = exe_dir.substr(0, exe_dir.find_last_of("\\"));

	install_dir_ = bin_dir.substr(0, bin_dir.find_last_of("\\") + 1);
}

namespace {

}

template <typename PluginType>
void WindowsBoot::get_plugin_xml_paths(std::vector<std::string>& paths)
{
	bfs::recursive_directory_iterator dir(install_dir_ + "share");
	bfs::recursive_directory_iterator end;
	for (; dir != end; ++dir) {
		bfs::path path = dir->path();

		if (path.extension() == ".xml") {
			paths.push_back(path.string());
		}
	}
}

void WindowsBoot::boot(csapex::PluginLocator* locator)
{
	locator->registerLocator<CorePlugin>(std::bind(&WindowsBoot::get_plugin_xml_paths<CorePlugin>, this, std::placeholders::_1));
	locator->registerLocator<MessageProvider>(std::bind(&WindowsBoot::get_plugin_xml_paths<MessageProvider>, this, std::placeholders::_1));
	locator->registerLocator<MessageRenderer>(std::bind(&WindowsBoot::get_plugin_xml_paths<MessageRenderer>, this, std::placeholders::_1));
	locator->registerLocator<NodeAdapterBuilder>(std::bind(&WindowsBoot::get_plugin_xml_paths<NodeAdapterBuilder>, this, std::placeholders::_1));
	locator->registerLocator<Node>(std::bind(&WindowsBoot::get_plugin_xml_paths<Node>, this, std::placeholders::_1));
	locator->registerLocator<DragIOHandler>(std::bind(&WindowsBoot::get_plugin_xml_paths<DragIOHandler>, this, std::placeholders::_1));
}
