/// HEADER
#include "detection.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

CSAPEX_REGISTER_CLASS(csapex::CornerDetection, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

CornerDetection::CornerDetection()
{
    Tag::createIfNotExists("Corners");
    addTag(Tag::get("Corners"));
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declareRange("k", 1.0, 400.0, 100.0, 1.0),
                 boost::bind(&CornerDetection::update, this));
    addParameter(param::ParameterFactory::declareRange("block size", 3, 31, 3, 2),
                 boost::bind(&CornerDetection::update, this));
    addParameter(param::ParameterFactory::declareRange("k size", 1, 31, 1, 2),
                 boost::bind(&CornerDetection::update, this));

}

void CornerDetection::allConnectorsArrived()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage);

    cv::Mat tmp;
    if(in->value.type() != CV_8UC1) {
        cv::cvtColor(in->value, tmp, CV_BGR2GRAY);
    } else {
        tmp = in->value;
    }

    cv::cornerHarris(tmp, tmp, block_size_, k_size_, k_);

    out->value = tmp.clone();
    output_->publish(out);
}

void CornerDetection::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Original");
    output_ = addOutput<CvMatMessage>("Corners");
    update();
}

void CornerDetection::update()
{
    k_ = param<double>("k");
    k_size_ = param<int>("k size");
    block_size_ = param<int>("block size");
}
