/// HEADER
#include "confidence_matrix_display.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/factory/message_factory.h>
#include <csapex/utility/timer.h>
#include <csapex/msg/any_message.h>

CSAPEX_REGISTER_CLASS(csapex::ConfidenceMatrixDisplay, csapex::Node)

using namespace csapex;

ConfidenceMatrixDisplay::ConfidenceMatrixDisplay()
    : connector_(nullptr)
{
}

void ConfidenceMatrixDisplay::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
}
void ConfidenceMatrixDisplay::process()
{
    connection_types::ConfidenceMatrixMessage::ConstPtr msg =
            msg::getMessage<connection_types::ConfidenceMatrixMessage>(connector_);

    confidence_ = msg->confidence;

    display_request();
}

const ConfidenceMatrix& ConfidenceMatrixDisplay::getConfidenceMatrix() const
{
    return confidence_;
}
