/// HEADER
#include "color_segmentation.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <boost/foreach.hpp>

CSAPEX_REGISTER_CLASS(csapex::ColorSegmentation, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ColorSegmentation::ColorSegmentation()
{
}

void ColorSegmentation::setParameterState(Memento::Ptr memento)
{
    Node::setParameterState(memento);
    loaded_state_ = boost::dynamic_pointer_cast<GenericState>(memento);
}

void ColorSegmentation::process()
{
    CvMatMessage::Ptr img = input_img_->getMessage<CvMatMessage>();

    bool encoding_changed = !img->getEncoding().matches(current_encoding);
    current_encoding = img->getEncoding();

    CvMatMessage::Ptr out_mask(new CvMatMessage(enc::mono, img->stamp));

    if(encoding_changed) {
        recompute();

        if(loaded_state_) {
            Node::setParameterState(loaded_state_);
            loaded_state_.reset((GenericState*)NULL);
            triggerParameterSetChanged();
            update();
        }

        triggerModelChanged();

        output_mask_->publish(out_mask);
        return;
    }

    cv::Mat bw;
    cv::inRange(img->value, min, max, bw);

    if(input_mask_->hasMessage()) {
        CvMatMessage::Ptr mask = input_mask_->getMessage<CvMatMessage>();
        cv::min(mask->value, bw, out_mask->value);
    } else {
        out_mask->value = bw;
    }

    output_mask_->publish(out_mask);
}

namespace {
std::string channelName(int idx, const Channel& c)
{
    std::stringstream name;
    name << "c" << idx << " [" << c.name << "]";
    return name.str();
}
}

void ColorSegmentation::update()
{
    min = cv::Scalar::all(0);
    max = cv::Scalar::all(0);

    for(std::size_t i = 0; i < current_encoding.channelCount(); ++i) {
        std::pair<int,int> val = readParameter<std::pair<int, int> >(channelName(i, current_encoding.getChannel(i)));

        min[i] = val.first;
        max[i] = val.second;
    }
}

void ColorSegmentation::recompute()
{
    setParameterSetSilence(true);
    removeTemporaryParameters();
    for(std::size_t i = 0; i < current_encoding.channelCount(); ++i) {
        Channel c = current_encoding.getChannel(i);

        std::string name = channelName(i, c);
        param::Parameter::Ptr p;
        if(c.fp) {
            p = param::ParameterFactory::declareInterval<double>(name, c.min_f, c.max_f, c.min_f, c.max_f, 1.0);
        } else {
            p = param::ParameterFactory::declareInterval<int>(name, c.min_i, c.max_i, c.min_i, c.max_i, 1);
        }
        addTemporaryParameter(p, boost::bind(&ColorSegmentation::update, this));
    }

    setParameterSetSilence(false);
    triggerParameterSetChanged();

    update();
}

void ColorSegmentation::setup()
{
    input_img_ = modifier_->addInput<CvMatMessage>("Image");
    input_mask_ = modifier_->addOptionalInput<CvMatMessage>("Mask");
    output_mask_ = modifier_->addOutput<CvMatMessage>("Mask");
}
