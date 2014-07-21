/// HEADER
#include "hog_extractor.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <utils_cv/color_functions.hpp>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision_features/descriptor_message.h>

/// SYSTEM
#include <opencv2/objdetect/objdetect.hpp>
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::HOGExtractor, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

HOGExtractor::HOGExtractor()
{
    addTag(Tag::get("vision_plugins"));
}

void HOGExtractor::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("gaussian sigma",
                                                       param::ParameterDescription("Standard deviation for Gaussian blur."),
                                                       0.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareBool("gamma correction",
                                                      param::ParameterDescription("Enable the gamma correction."),
                                                      true));
    addParameter(param::ParameterFactory::declareRange("orientation bins",
                                                       param::ParameterDescription("Amount of histogram bins per block."),
                                                       2, 18, 9, 1));
}

void HOGExtractor::setup()
{
    in_  = modifier_->addInput<CvMatMessage>("image");
    out_ = modifier_->addOutput<DescriptorMessage>("descriptor");
}

void HOGExtractor::process()
{
    CvMatMessage::Ptr      in = in_->getMessage<CvMatMessage>();
    DescriptorMessage::Ptr out(new DescriptorMessage);

    if(in->getEncoding() != enc::mono)
        throw std::runtime_error("Images must be mono!");

    cv::Mat &value = in->value;

    if(value.rows < HOG_BLOCK_HEIGHT)
        throw std::runtime_error("Images have height at least 16px!");
    if(value.cols < HOG_BLOCK_WIDTH)
        throw std::runtime_error("Images have width at least 16px!");
    if(value.rows % HOG_CELL_HEIGHT != 0)
        throw std::runtime_error("Images have height as multiple of 8px!");
    if(value.cols % HOG_CELL_WIDTH != 0)
        throw std::runtime_error("Images have width as multiple of 8px!");

    double gauss = param<double>("gaussian sigma");
    bool   gamma = param<bool>("gamma correction");
    int    bins   = param<int>("orientation bins");
    std::vector<float> desc;
    cv::HOGDescriptor d(cv::Size(value.cols, value.rows),
                        cv::Size(HOG_BLOCK_WIDTH, HOG_BLOCK_HEIGHT),
                        cv::Size(HOG_CELL_WIDTH, HOG_CELL_HEIGHT),
                        cv::Size(HOG_CELL_WIDTH, HOG_CELL_HEIGHT),
                        bins,
                        0.2,
                        gauss == 0.0 ? -1 : gauss,
                        gamma);
    d.compute(value, desc);
    out->value = cv::Mat(desc, true);

    out_->publish(out);
}


