/// HEADER
#include <csapex_vision/filter.h>

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <QLabel>

using namespace csapex;
using namespace connection_types;


Filter::Filter()
    : input_img_(NULL), input_mask_(NULL), output_img_(NULL), output_mask_(NULL)
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
    CvMatMessage::Ptr img_msg = input_img_->getClonedMessage<CvMatMessage>();
    if(img_msg.get() && !img_msg->value.empty()) {
        CvMatMessage::Ptr mask_msg;
        cv::Mat mask;
        if(usesMask()) {
            if(input_mask_->hasMessage()) {
                mask_msg = input_mask_->getClonedMessage<CvMatMessage>();
            } else {
                mask_msg.reset(new CvMatMessage(enc::mono, img_msg->stamp));
            }
            mask = mask_msg->value;
        }

        filter(img_msg->value, mask);

        output_img_->publish(img_msg);
        if(usesMask() && mask_msg != nullptr) {
            output_mask_->publish(mask_msg);
        }
    }
}

bool Filter::usesMask()
{
    return true;
}
