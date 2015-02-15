/// HEADER
#include <csapex_vision/filter.h>

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <QLabel>

using namespace csapex;
using namespace connection_types;


Filter::Filter()
    : input_img_(nullptr), input_mask_(nullptr), output_img_(nullptr), output_mask_(nullptr)
{
}

Filter::~Filter()
{
}

void Filter::setup()
{
    input_img_ = modifier_->addInput<CvMatMessage>("Image");
    if(usesMask()) {
        input_mask_ = modifier_->addOptionalInput<CvMatMessage>("Mask");
    }
    output_img_ = modifier_->addOutput<CvMatMessage>("Image");
    if(usesMask()) {
        output_mask_ = modifier_->addOutput<CvMatMessage>("Mask");
    }
}

void Filter::process()
{
    CvMatMessage::Ptr img_msg = msg::getClonedMessage<CvMatMessage>(input_img_);
    if(img_msg.get() && !img_msg->value.empty()) {
        CvMatMessage::Ptr mask_msg;
        cv::Mat mask;
        if(usesMask()) {
            if(msg::hasMessage(input_mask_)) {
                mask_msg = msg::getClonedMessage<CvMatMessage>(input_mask_);
            } else {
                mask_msg.reset(new CvMatMessage(enc::mono, img_msg->stamp_micro_seconds));
            }
            mask = mask_msg->value;
        }

        filter(img_msg->value, mask);

        msg::publish(output_img_, img_msg);
        if(usesMask() && mask_msg != nullptr) {
            msg::publish(output_mask_, mask_msg);
        }
    }
}

bool Filter::usesMask()
{
    return true;
}
