/// HEADER
#include "output_display.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/manager/message_renderer_manager.h>
#include <csapex/msg/any_message.h>
#include <csapex/model/node_handle.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/io/raw_message.h>

/// SYSTEM
#include <QBuffer>
#include <QImageWriter>

CSAPEX_REGISTER_CLASS(csapex::OutputDisplay, csapex::Node)


using namespace csapex;
using namespace connection_types;

OutputDisplay::OutputDisplay()
    : jpg_quality_(70)
{
}

OutputDisplay::~OutputDisplay()
{
}
void OutputDisplay::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<AnyMessage>("Message");
}

void OutputDisplay::setupParameters(Parameterizable &params)
{
    params.addHiddenParameter(param::ParameterFactory::declareRange("jpg/quality", 0, 100, 70, 1),
                              jpg_quality_);
}

void OutputDisplay::process()
{
    bool has_direct_adapter = display_request.isConnected();
    bool has_adapter = node_handle_->raw_data_connection.isConnected();

    bool has_any_adapter = has_direct_adapter || has_adapter;

    if(!has_any_adapter) {
//        ainfo << "no head at "<< node_handle_->getUUID() << " / " << (long) node_handle_.get() << std::endl; // TODO: debug info, remove when all works well
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
        if(has_adapter) {
            QBuffer buffer;
            QImageWriter writer(&buffer, "JPG");
            writer.setQuality(jpg_quality_);
            writer.write(img);

            std::shared_ptr<RawMessage> msg = std::make_shared<RawMessage>(buffer.data().data(), buffer.size(),
                                                                           getUUID().getAbsoluteUUID());
            node_handle_->raw_data_connection(msg);

        }

        if(has_direct_adapter){
            display_request(img);
        }
    }
}
