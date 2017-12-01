/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>
#include <csapex/factory/generic_node_factory.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/factory/node_factory_impl.h>

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex_opencv/yaml_io.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::RegisterVisionFeaturePlugin, csapex::CorePlugin)

using namespace csapex;

RegisterVisionFeaturePlugin::RegisterVisionFeaturePlugin()
{
}

void rotateKeypoint(const connection_types::GenericValueMessage<cv::KeyPoint>& input,
                    connection_types::GenericValueMessage<cv::KeyPoint>& output)
{
    output = input;
}

void RegisterVisionFeaturePlugin::init(CsApexCore& core)
{
    Tag::createIfNotExists("Features");

    auto c = GenericNodeFactory::createConstructorFromFunction(rotateKeypoint, "rotateKeypoint");
    c->setDescription("Test function for rotating a keypoint");
    core.getNodeFactory()->registerNodeType(c);
}
