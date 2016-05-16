/// HEADER
#include "output_display.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/manager/message_renderer_manager.h>
#include <csapex/msg/any_message.h>

CSAPEX_REGISTER_CLASS(csapex::OutputDisplay, csapex::Node)


using namespace csapex;
using namespace connection_types;

OutputDisplay::OutputDisplay()
    : adapted_(false)
{
}

OutputDisplay::~OutputDisplay()
{
}

void OutputDisplay::setAdapted()
{
    adapted_ = true;
}

void OutputDisplay::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<AnyMessage>("Message");
}

void OutputDisplay::process()
{
    if(!adapted_) {
        return;
    }

    TokenData::ConstPtr msg = msg::getMessage<TokenData>(input_);

    MessageRenderer::Ptr renderer = MessageRendererManager::instance().createMessageRenderer(msg);
    if(!renderer) {
        return;
    }

    if(renderer != renderer_) {
        renderer_ = renderer;
        setTemporaryParameters(renderer_->getParameters());
    }

    QImage img = renderer->render(msg);

    if(!img.isNull()) {
        display_request(img);
    }
}
