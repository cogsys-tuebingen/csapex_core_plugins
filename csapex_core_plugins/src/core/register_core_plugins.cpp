/// HEADER
#include "register_core_plugins.h"

/// COMPONENT
#include "../io/file_importer.h"

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_state.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/yaml_io.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::RegisterCorePlugins, csapex::CorePlugin)

using namespace csapex;


RegisterCorePlugins::RegisterCorePlugins()
{
}

void RegisterCorePlugins::init(CsApexCore& core)
{
    Tag::createIfNotExists("Buffer");
    Tag::createIfNotExists("General");
    Tag::createIfNotExists("Input");
    Tag::createIfNotExists("Output");
    Tag::createIfNotExists("RosIO");
    Tag::createIfNotExists("ConsoleIO");
    Tag::createIfNotExists("Debug");


    connection_types::GenericVectorMessage::registerType<int>();
    connection_types::GenericVectorMessage::registerType<double>();
    connection_types::GenericVectorMessage::registerType<std::string>();
}

void RegisterCorePlugins::shutdown()
{
}
