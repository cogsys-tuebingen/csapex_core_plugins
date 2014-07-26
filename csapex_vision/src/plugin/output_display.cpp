/// HEADER
#include "output_display.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <utils_qt/QtCvImageConverter.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

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
    input_ = modifier_->addInput<CvMatMessage>("Image", false, true);
}

void OutputDisplay::process()
{
    CvMatMessage::Ptr mat_msg = input_->getMessage<CvMatMessage>();

    if(mat_msg.get() && !mat_msg->value.empty()) {
        QSharedPointer<QImage> img = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mat_msg->value);
        display_request(img);
    }
}
