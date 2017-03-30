/// HEADER
#include "adaptive_threshold.h"

/// PROJECT
#include <csapex_opencv/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::AdaptiveThreshold, csapex::Node)

using namespace csapex;
using namespace connection_types;

AdaptiveThreshold::AdaptiveThreshold()
{
}


void AdaptiveThreshold::process()
{
    CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(input_);

    if(!img->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    double maxValue = readParameter<double>("maxValue");
    int adaptiveMethod = readParameter<int>("adaptiveMethod");
    int thresholdType = readParameter<int>("thresholdType");
    int blockSize = readParameter<int>("blockSize");
    double C = readParameter<double>("C");

    if((blockSize % 2) == 0) {
        --blockSize;
    }

    CvMatMessage::Ptr out(new CvMatMessage(enc::mono, img->frame_id, img->stamp_micro_seconds));

    cv::adaptiveThreshold(img->value, out->value, maxValue, adaptiveMethod, thresholdType, blockSize, C);

    msg::publish(output_, out);
}


void AdaptiveThreshold::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");

    output_ = node_modifier.addOutput<CvMatMessage>("thresholded");
}

void AdaptiveThreshold::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("maxValue", 1.0, 255.0, 100.0, 0.1));

    std::map<std::string, int > adaptiveMethod;
    adaptiveMethod["ADAPTIVE_THRESH_MEAN_C"] = (int) cv::ADAPTIVE_THRESH_MEAN_C;
    adaptiveMethod["ADAPTIVE_THRESH_GAUSSIAN_C"] = (int) cv::ADAPTIVE_THRESH_GAUSSIAN_C;
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet<int>("adaptiveMethod", adaptiveMethod, (int) cv::ADAPTIVE_THRESH_MEAN_C));

    std::map<std::string, int> thresholdType;
    thresholdType["THRESH_BINARY"] = (int) cv::THRESH_BINARY;
    thresholdType["THRESH_BINARY_INV"] = (int) cv::THRESH_BINARY_INV;
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet<int>("thresholdType", thresholdType, (int) cv::THRESH_BINARY));

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("blockSize", 3, 1001, 3, 2));

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("C", -255.0, 255.0, 0.0, 0.1));
}
