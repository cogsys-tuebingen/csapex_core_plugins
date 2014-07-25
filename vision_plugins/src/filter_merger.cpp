#include "filter_merger.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Merger, csapex::Node)

using namespace csapex;
using namespace connection_types;

Merger::Merger()
{
    addParameter(param::ParameterFactory::declareRange("input count", 2, MERGER_INPUT_MAX, 2, 1), boost::bind(&Merger::updateInputs, this));
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
    CvMatMessage::Ptr out_msg(new CvMatMessage(encoding));
    out_msg->value = out_img;
    output_->publish(out_msg);
}

void Merger::updateInputs()
{
    int input_count = readParameter<int>("input count");
    int current_amount = countInputs();

    if(current_amount > input_count) {
        for(int i = current_amount; i > input_count ; i--) {
            ConnectorIn* in = getInput(i - 1);
            if(in->isConnected()) {
                in->disable();
            } else {
                removeInput(in);
            }
        }
    } else {
        int to_add = input_count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            getInput(i)->enable();
        }
        for(int i = 0 ; i < to_add ; i++) {
            modifier_->addInput<CvMatMessage>("Channel", true);
        }
    }

}

void Merger::collectMessage(std::vector<cv::Mat> &messages, Encoding& encoding)
{
    for(int i = 0 ; i < countInputs() ; i++) {
        ConnectorIn *in = getInput(i);
        if(in->isConnected()) {
            CvMatMessage::Ptr msg = in->getMessage<CvMatMessage>();
            in->setLabel(msg->getEncoding().toString());
            messages.push_back(msg->value);
            encoding.insert(encoding.end(), msg->getEncoding().begin(), msg->getEncoding().end());
        }
    }
}
