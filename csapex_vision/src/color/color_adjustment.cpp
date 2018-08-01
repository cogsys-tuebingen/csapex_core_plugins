/// HEADER
#include "color_adjustment.h"

/// COMPONENT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <cslibs_vision/utils/histogram.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/generic_state.h>

CSAPEX_REGISTER_CLASS(csapex::ColorAdjustment, csapex::Node)

using namespace csapex;
using namespace connection_types;

ColorAdjustment::ColorAdjustment() :
    active_preset_(NONE)
{
}

void ColorAdjustment::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("adjusted");
}

void ColorAdjustment::setupParameters(Parameterizable& parameters)
{

    std::map<std::string, int> presets = {
        {"HSV", (int) HSV},
        {"HSL", (int) HSL},
        {"STD", (int) STD}
    };
    parameters.addParameter(csapex::param::factory::declareBool("normalize", false));
    parameters.addParameter(csapex::param::factory::declareRange("lightness", -255, 255, 0, 1));

    parameters.addParameter(csapex::param::factory::declareParameterSet("preset", presets, (int) HSV),
                            std::bind(&ColorAdjustment::setPreset, this));
}

namespace {
std::string channelName(int idx, const Channel& c)
{
    std::stringstream name;
    name << "c" << idx << " [" << c.name << "]";
    return name.str();
}
}

void ColorAdjustment::setParameterState(GenericStatePtr memento)
{
    Node::setParameterState(memento);
    loaded_state_ = std::dynamic_pointer_cast<GenericState>(memento);
}


void ColorAdjustment::process()
{
    CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(input_);

    bool encoding_changed = !img->getEncoding().matches(current_encoding);
    current_encoding = img->getEncoding();

    if(encoding_changed) {
        recompute();

        if(loaded_state_) {
            Node::setParameterState(loaded_state_);
            loaded_state_.reset((GenericState*)nullptr);
            triggerParameterSetChanged();
            update();
        }

        return;
    }


    std::vector<cv::Mat> channels;
    cv::split(img->value, channels);

    bool normalize = readParameter<bool>("normalize");

    for(unsigned i = 0 ; i < channels.size() ; i++) {
        double min = mins[i];
        double max = maxs[i];
        if(normalize) {
            cv::normalize(channels[i], channels[i], max, min, cv::NORM_MINMAX);
        } else {
            cv::threshold(channels[i], channels[i], min, max, cv::THRESH_TOZERO);
            cv::threshold(channels[i], channels[i], max, max, cv::THRESH_TOZERO_INV);
        }

    }

    CvMatMessage::Ptr result(new CvMatMessage(img->getEncoding(), img->frame_id, img->stamp_micro_seconds));
    cv::merge(channels, result->value);
    addLightness(result->value);

    msg::publish(output_, result);
}

void ColorAdjustment::recompute()
{
    setParameterSetSilence(true);
    removeTemporaryParameters();
    for(std::size_t i = 0; i < current_encoding.channelCount(); ++i) {
        Channel c = current_encoding.getChannel(i);

        std::string name = channelName(i, c);
        csapex::param::Parameter::Ptr p;
        if(c.fp) {
            p = csapex::param::factory::declareInterval<double>(name, c.min_f, c.max_f, c.min_f, c.max_f, 1.0);
        } else {
            p = csapex::param::factory::declareInterval<int>(name, c.min_i, c.max_i, c.min_i, c.max_i, 1);
        }
        addTemporaryParameter(p, std::bind(&ColorAdjustment::update, this));
    }

    setParameterSetSilence(false);
    triggerParameterSetChanged();

    update();
}

void ColorAdjustment::update()
{
    mins.clear();
    maxs.clear();

    for(std::size_t i = 0; i < current_encoding.channelCount(); ++i) {
        std::pair<int,int> val = readParameter<std::pair<int, int> >(channelName(i, current_encoding.getChannel(i)));

        mins.push_back(val.first);
        maxs.push_back(val.second);
    }
}

void ColorAdjustment::setPreset()
{
    active_preset_  = static_cast<Preset> (readParameter<int>("preset"));
}

void ColorAdjustment::addLightness(cv::Mat &img)
{
    double param_lightness = readParameter<int>("lightness");
    double abs_lightness = std::abs(param_lightness);

    cv::Scalar  cv_value;
    cv_value[0] = abs_lightness;
    cv_value[1] = active_preset_ == HSL || active_preset_ == HSV ? 0 : abs_lightness;
    cv_value[2] = active_preset_ == HSL || active_preset_ == HSV ? 0 : abs_lightness;
    cv_value[3] = abs_lightness;

    cv::Mat     lightness(img.rows, img.cols, img.type(), cv_value);
    if(param_lightness < 0) {
        cv::subtract(img, lightness, img);
    } else if(param_lightness > 0) {
        cv::add(img, lightness, img);
    }
}
