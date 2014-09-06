/// HEADER
#include "adaptive_threshold.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
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
    CvMatMessage::Ptr img = input_->getMessage<CvMatMessage>();

    if(img->hasChannels(1, CV_8U)) {
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

    CvMatMessage::Ptr out(new CvMatMessage(enc::mono));

    cv::adaptiveThreshold(img->value, out->value, maxValue, adaptiveMethod, thresholdType, blockSize, C);

    output_->publish(out);
}


void AdaptiveThreshold::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("original");

    output_ = modifier_->addOutput<CvMatMessage>("thresholded");
}

void AdaptiveThreshold::setupParameters()
{
    addParameter(param::ParameterFactory::declare("maxValue", 1.0, 255.0, 100.0, 0.1));

    std::vector< std::pair<std::string, int> > adaptiveMethod;
    adaptiveMethod.push_back(std::make_pair("ADAPTIVE_THRESH_MEAN_C", (int) cv::ADAPTIVE_THRESH_MEAN_C));
    adaptiveMethod.push_back(std::make_pair("ADAPTIVE_THRESH_GAUSSIAN_C", (int) cv::ADAPTIVE_THRESH_GAUSSIAN_C));
    addParameter(param::ParameterFactory::declareParameterSet<int>("adaptiveMethod", adaptiveMethod));

    std::vector< std::pair<std::string, int> > thresholdType;
    thresholdType.push_back(std::make_pair("THRESH_BINARY", (int) cv::THRESH_BINARY));
    thresholdType.push_back(std::make_pair("THRESH_BINARY_INV", (int) cv::THRESH_BINARY_INV));
    addParameter(param::ParameterFactory::declareParameterSet<int>("thresholdType", thresholdType));

    addParameter(param::ParameterFactory::declare("blockSize", 3, 1001, 3, 2));

    addParameter(param::ParameterFactory::declare("C", -255.0, 255.0, 0.0, 0.1));
}
