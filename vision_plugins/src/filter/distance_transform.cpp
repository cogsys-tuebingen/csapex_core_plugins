/// HEADER
#include "distance_transform.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::DistanceTransform, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


DistanceTransform::DistanceTransform()
{
}

void DistanceTransform::setupParameters()
{

    std::map<std::string, int > distanceType;
    distanceType["CV_DIST_L1"] = (int) CV_DIST_L1;
    distanceType["CV_DIST_L2"] = (int) CV_DIST_L2;
    distanceType["CV_DIST_C"] = (int) CV_DIST_C;

    addParameter(param::ParameterFactory::declareParameterSet<int>("distanceType", distanceType, (int) CV_DIST_L1));


    std::map<std::string, int > maskSize;
    maskSize["3"] = 3;
    maskSize["4"] = 5;
    maskSize["CV_DIST_MASK_PRECISE"] = (int) CV_DIST_MASK_PRECISE;

    addParameter(param::ParameterFactory::declareParameterSet<int>("maskSize", maskSize, 3));


    std::map<std::string, int > labelType;
    labelType["DIST_LABEL_CCOMP"] = (int) cv::DIST_LABEL_CCOMP;
    labelType["DIST_LABEL_PIXEL"] = (int) cv::DIST_LABEL_PIXEL;

    addParameter(param::ParameterFactory::declareParameterSet<int>("labelType", labelType, (int) cv::DIST_LABEL_CCOMP));
}

void DistanceTransform::setup()
{
    in_ = modifier_->addInput<CvMatMessage>("original");

    out_ = modifier_->addOutput<CvMatMessage>("thresholded");
    out_label_ = modifier_->addOutput<CvMatMessage>("lables");
}

void DistanceTransform::process()
{
    CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(in_);

    if(!img->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    CvMatMessage::Ptr out(new CvMatMessage(enc::mono, img->stamp_micro_seconds));
    CvMatMessage::Ptr labels(new CvMatMessage(enc::unknown, img->stamp_micro_seconds));

    cv::distanceTransform(img->value, out->value, labels->value,
                          readParameter<int>("distanceType"),
                          readParameter<int>("maskSize"),
                          readParameter<int>("labelType"));

    msg::publish(out_, out);
    msg::publish(out_label_, labels);
}

