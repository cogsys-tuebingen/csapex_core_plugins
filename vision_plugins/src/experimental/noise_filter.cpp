/// HEADER
#include "noise_filter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::NoiseFilter, csapex::Node)

NoiseFilter::NoiseFilter()
{
    Tag::createIfNotExists("Experimental");
    addTag(Tag::get("Experimental"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("randomized", RANDOM)
            ("temporal", TEMPORAL);

    addParameter(param::ParameterFactory::declareParameterSet("type", types),
                 boost::bind(&NoiseFilter::update, this));
    addParameter(param::ParameterFactory::declareRange("min mutation prob", 0.0, 1.0, 0.5, 0.05),
                 boost::bind(&NoiseFilter::update, this));

}

void NoiseFilter::process()
{
    CvMatMessage::Ptr in(input_->getMessage<CvMatMessage>());
    CvMatMessage::Ptr out(new CvMatMessage(in->getEncoding()));

    if(in->getEncoding() != enc::mono) {
        throw std::runtime_error("Encoding must be mono!");
    }

    noise_filter_->filter(in->value, out->value);
    output_->publish(out);

//    if(probs_out_->isConnected()) {
//#warning "FIX ENCODING"
//        CvMatMessage::Ptr probs(new CvMatMessage(enc::unknown));
//        noise_filter_->getProbabilities(probs->value);
//        probs_out_->publish(probs);
//    }

}

void NoiseFilter::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("unfiltered");
    output_ = modifier_->addOutput<CvMatMessage>("filtered");
    probs_out_ = modifier_->addOutput<CvMatMessage>("probabilities");

    update();
}

void NoiseFilter::update()
{
    float min_mutation_prob = param<double>("min mutation prob");
    Type type               = (Type) param<int>("type");

    switch(type) {
    case RANDOM:
        noise_filter_.reset(new utils_cv::RandomizedNoiseFilter(min_mutation_prob));
        break;
    case TEMPORAL:
        noise_filter_.reset(new utils_cv::TheThingFilter);
        break;
    default:
        throw std::runtime_error("Unknown filter type!");
    }
}
