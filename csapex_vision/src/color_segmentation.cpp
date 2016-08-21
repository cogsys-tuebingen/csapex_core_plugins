/// HEADER
#include "color_segmentation.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/profiling/timer.h>

/// SYSTEM


CSAPEX_REGISTER_CLASS(csapex::ColorSegmentation, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ColorSegmentation::ColorSegmentation()
    : loaded_(false)
{
}

void ColorSegmentation::setParameterState(Memento::Ptr memento)
{
    Node::setParameterState(memento);
    loaded_state_ = std::dynamic_pointer_cast<GenericState>(memento);
}

void ColorSegmentation::process()
{
    CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(input_img_);

    bool encoding_changed = !img->getEncoding().matches(current_encoding);
    current_encoding = img->getEncoding();

    CvMatMessage::Ptr out_mask(new CvMatMessage(enc::mono, img->stamp_micro_seconds));

    if(encoding_changed) {
        if(!loaded_) {
            updateParameters();

            if(loaded_state_) {
                Node::setParameterState(loaded_state_);
                loaded_state_.reset();
                triggerParameterSetChanged();
                update();
            }
        } else {
            loaded_ = false;
        }
    }

    {
        INTERLUDE("update");
        update();
    }

    cv::Mat bw;
    cv::inRange(img->value, min, max, bw);

    if(msg::hasMessage(input_mask_)) {
        CvMatMessage::ConstPtr mask = msg::getMessage<CvMatMessage>(input_mask_);
        cv::min(mask->value, bw, out_mask->value);
    } else {
        out_mask->value = bw;
    }

    apex_assert(!out_mask->value.empty());
    msg::publish(output_mask_, out_mask);
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

void ColorSegmentation::updateParameters()
{
    setParameterSetSilence(true);
    removeTemporaryParameters();
    for(std::size_t i = 0; i < current_encoding.channelCount(); ++i) {
        Channel c = current_encoding.getChannel(i);

        std::string name = channelName(i, c);
        csapex::param::Parameter::Ptr p;
        if(c.fp) {
            p = csapex::param::ParameterFactory::declareInterval<double>(name, c.min_f, c.max_f, c.min_f, c.max_f, 1.0);
        } else {
            p = csapex::param::ParameterFactory::declareInterval<int>(name, c.min_i, c.max_i, c.min_i, c.max_i, 1);
        }
        addTemporaryParameter(p, std::bind(&ColorSegmentation::update, this));
    }

    setParameterSetSilence(false);
    triggerParameterSetChanged();
}

void ColorSegmentation::setup(NodeModifier& node_modifier)
{
    input_img_ = node_modifier.addInput<CvMatMessage>("Image");
    input_mask_ = node_modifier.addOptionalInput<CvMatMessage>("Mask");
    output_mask_ = node_modifier.addOutput<CvMatMessage>("Mask");
}

void ColorSegmentation::setupParameters(Parameterizable &params)
{
    if(loaded_state_) {
        for(auto it = loaded_state_->params.begin(); it != loaded_state_->params.end(); ++it) {
            csapex::param::Parameter::Ptr p = it->second;
            params.addParameter(param::ParameterFactory::clone(p));
        }
        Node::setParameterState(loaded_state_);
        loaded_ = true;
        loaded_state_.reset();
        triggerParameterSetChanged();
        update();
    }
}
