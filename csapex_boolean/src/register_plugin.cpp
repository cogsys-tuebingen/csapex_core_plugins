/// HEADER
#include "register_plugin.h"

/// PROJECT
#include <csapex/model/tag.h>
#include <csapex/factory/message_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::RegisterPlugin, csapex::CorePlugin)

using namespace csapex;
using namespace boolean;

RegisterPlugin::RegisterPlugin()
{
}


void RegisterPlugin::prepare(Settings&)
{
    Tag::createIfNotExists("Boolean");
}
