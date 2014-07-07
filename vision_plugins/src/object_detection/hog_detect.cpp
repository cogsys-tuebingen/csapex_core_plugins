/// HEADER
#include "hog_detect.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <boost/assign.hpp>
#include <opencv2/objdetect/objdetect.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::HOGDetect, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

HOGDetect::HOGDetect()
{
    addTag(Tag::get("vision_plugins"));
}

HOGDetect::~HOGDetect()
{
}

void HOGDetect::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("thresh", -10.0, 10.0, 0.0, 0.1));
    std::map<std::string, int> types = boost::assign::map_list_of
            ("default", DEFAULT)
            ("daimler", DAIMLER);
    addParameter(param::ParameterFactory::declareParameterSet("type", types));
}

void HOGDetect::setup()
{
    in_  = modifier_->addInput<CvMatMessage>("image");
    out_ = modifier_->addOutput<CvMatMessage>("detections");
}

void HOGDetect::process()
{
    CvMatMessage::Ptr in = in_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr out(new CvMatMessage(enc::bgr));
    cv::Mat working_copy;

    if(in->value.type() == CV_8UC1) {
        cv::cvtColor(in->value, out->value, CV_GRAY2BGR);
        working_copy = in->value.clone();
    } else if(in->value.type() == CV_8UC3) {
        cv::cvtColor(in->value, working_copy, CV_BGR2GRAY);
        out->value = in->value.clone();
    } else {
        throw std::runtime_error("Need grayscale or bgr!");
    }

    cv::HOGDescriptor h;
    std::vector<cv::Rect> locations;

    double th = param<double>("thresh");
    Type   t  = (Type) param<int>("type");


    if(t == DEFAULT) {
        h.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    } else {
        h.winSize = cv::Size(48, 96);
        h.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
    }

    h.detectMultiScale(working_copy, locations, th);

    for(std::vector<cv::Rect>::iterator
        it = locations.begin() ;
        it != locations.end() ;
        ++it ) {
        cv::rectangle(out->value, *it, cv::Scalar(255,0,255), 1 , CV_AA);
    }

    std::cout << "count " << locations.size() << std::endl;

    out_->publish(out);
}

