#include "filter_merger.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Merger, csapex::Node)

using namespace csapex;
using namespace connection_types;

Merger::Merger()
{
    addParameter(param::ParameterFactory::declareRange("input count", 2, MERGER_INPUT_MAX, 2, 1), std::bind(&Merger::updateInputs, this));
}

void Merger::setup()
{
    output_ = modifier_->addOutput<CvMatMessage>("Merged Image");

    updateInputs();
}

void Merger::process()
{
    std::vector<cv::Mat> msgs;
    Encoding encoding;
    collectMessage(msgs, encoding);
    cv::Mat out_img;

    cv::merge(msgs, out_img);
    CvMatMessage::Ptr out_msg(new CvMatMessage(encoding, stamp_));
    out_msg->value = out_img;
    msg::publish(output_, out_msg);
}

void Merger::updateInputs()
{
    int input_count = readParameter<int>("input count");

    std::vector<Input*> inputs = modifier_->getMessageInputs();
    int current_amount = inputs.size();

    if(current_amount > input_count) {
        for(int i = current_amount; i > input_count ; i--) {
            Input* in = inputs[i - 1];
            if(msg::isConnected(in)) {
                msg::disable(in);
            } else {
                modifier_->removeInput(msg::getUUID(in));
            }
        }
    } else {
        int to_add = input_count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            msg::enable(inputs[i]);
        }
        for(int i = 0 ; i < to_add ; i++) {
            modifier_->addOptionalInput<CvMatMessage>("Channel");
        }
    }

}

void Merger::collectMessage(std::vector<cv::Mat> &messages, Encoding& encoding)
{
    bool first = true;
    std::vector<Input*> inputs = modifier_->getMessageInputs();
    for(std::size_t i = 0 ; i < inputs.size() ; i++) {
        Input *in = inputs[i];
        if(msg::hasMessage(in)) {
            CvMatMessage::ConstPtr msg = msg::getMessage<CvMatMessage>(in);
            if(first) {
                stamp_ = msg->stamp_micro_seconds;
                first = false;
            }
            msg::setLabel(in, msg->getEncoding().toString());
            messages.push_back(msg->value);
            encoding.append(msg->getEncoding());
        }
    }
}

void Merger::stateChanged()
{
    updateInputs();
}
