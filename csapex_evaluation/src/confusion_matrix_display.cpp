/// HEADER
#include "confusion_matrix_display.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/factory/message_factory.h>
#include <csapex/utility/timer.h>

#include <csapex/msg/any_message.h>
CSAPEX_REGISTER_CLASS(csapex::ConfusionMatrixDisplay, csapex::Node)

using namespace csapex;

ConfusionMatrixDisplay::ConfusionMatrixDisplay()
    : connector_(nullptr)
{
}

void ConfusionMatrixDisplay::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
}
void ConfusionMatrixDisplay::process()
{
    connection_types::ConfusionMatrixMessage::ConstPtr msg = msg::getMessage<connection_types::ConfusionMatrixMessage>(connector_);

    {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        confusion_ = msg->confusion;
    }

    display_request();
}

const ConfusionMatrix& ConfusionMatrixDisplay::getConfusionMatrix() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return confusion_;
}
