/// HEADER
#include "confusion_matrix_display.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/message_factory.h>
#include <csapex/utility/timer.h>

CSAPEX_REGISTER_CLASS(csapex::ConfusionMatrixDisplay, csapex::Node)

using namespace csapex;

ConfusionMatrixDisplay::ConfusionMatrixDisplay()
    : connector_(NULL)
{
}

void ConfusionMatrixDisplay::setup()
{
    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}
void ConfusionMatrixDisplay::process()
{
    connection_types::ConfusionMatrixMessage::Ptr msg = connector_->getMessage<connection_types::ConfusionMatrixMessage>();

    confusion_ = msg->confusion;

    display_request();
}

const ConfusionMatrix& ConfusionMatrixDisplay::getConfusionMatrix() const
{
    return confusion_;
}
