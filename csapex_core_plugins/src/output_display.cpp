/// HEADER
#include "output_display.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/manager/message_renderer_manager.h>

CSAPEX_REGISTER_CLASS(csapex::OutputDisplay, csapex::Node)


using namespace csapex;
using namespace connection_types;

OutputDisplay::OutputDisplay()
{
}

OutputDisplay::~OutputDisplay()
{
}

void OutputDisplay::setup()
{
    input_ = modifier_->addInput<AnyMessage>("Message");
}

void OutputDisplay::process()
{
    ConnectionType::Ptr msg = input_->getMessage<ConnectionType>();

    MessageRenderer::Ptr renderer = MessageRendererManager::instance().createMessageRenderer(msg);

    if(renderer != renderer_) {
        renderer_ = renderer;
        setTemporaryParameters(renderer_->getParameters());
    }

    QSharedPointer<QImage> img = renderer->render(msg);

    if(!img.isNull()) {
        display_request(img);
    }
}
