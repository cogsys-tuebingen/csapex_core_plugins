#include "merge.h"

/// PROJECT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Merge, csapex::Node)

using namespace csapex;
using namespace connection_types;

Merge::Merge()
    : VariadicInputs(connection_types::makeEmpty<CvMatMessage>())
{
}

void Merge::setupParameters(Parameterizable &parameters)
{
    setupVariadicParameters(parameters);
}

void Merge::setup(NodeModifier& node_modifier)
{
    setupVariadic(node_modifier);

    output_ = node_modifier.addOutput<CvMatMessage>("Merged Image");
}

Input* Merge::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool /*optional*/)
{
    return VariadicInputs::createVariadicInput(type, label.empty() ? "Channel" : label, true);
}

void Merge::process()
{
    std::vector<cv::Mat> msgs;
    Encoding encoding;
    collectMessage(msgs, encoding);
    cv::Mat out_img;

    if(msgs.empty()) {
        return;
    }

    cv::merge(msgs, out_img);
    CvMatMessage::Ptr out_msg(new CvMatMessage(encoding, stamp_));
    out_msg->value = out_img;
    msg::publish(output_, out_msg);
}

void Merge::collectMessage(std::vector<cv::Mat> &messages, Encoding& encoding)
{
    bool first = true;
    std::vector<Input*> inputs = node_modifier_->getMessageInputs();
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
