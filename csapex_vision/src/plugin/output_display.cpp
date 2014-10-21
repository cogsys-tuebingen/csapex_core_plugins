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
    input_ = modifier_->addInput<CvMatMessage>("Image");
}

void OutputDisplay::process()
{
    CvMatMessage::Ptr mat_msg = input_->getMessage<CvMatMessage>();

    if(mat_msg.get() && !mat_msg->value.empty()) {

        Encoding encoding = mat_msg->getEncoding();

        QSharedPointer<QImage> img;
        if(encoding.matches(enc::rgb)) {
            cv::Mat mat = mat_msg->value;
            img = QSharedPointer<QImage>(new QImage((uchar*) mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888));

        } else if(encoding.matches(enc::bgr) || encoding.matches(enc::mono)) {
            img = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mat_msg->value);

        } else {
            return;
        }

        display_request(img);
    }
}
