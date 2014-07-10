/// HEADER
#include "hog_detect.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <fstream>

/// SYSTEM
#include <boost/assign.hpp>

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
    std::map<std::string, int> det_types = boost::assign::map_list_of
            ("single scale", SINGLE_SCALE)
            ("multi scale", MULTI_SCALE);
    addParameter(param::ParameterFactory::declareParameterSet("detection type", det_types));

    std::map<std::string, int> svm_types = boost::assign::map_list_of
            ("default", DEFAULT)
            ("daimler", DAIMLER)
            ("custom", CUSTOM);
    addParameter(param::ParameterFactory::declareParameterSet("svm type", svm_types));

    addParameter(param::ParameterFactory::declareFileInputPath("svm path",""));
    setParameterEnabled("svm path", false);
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
    if(in->getEncoding() != enc::mono)
        throw std::runtime_error("Need grayscale or bgr!");


    cv::HOGDescriptor h;

    double        det_threshold = param<double>("thresh");
    DetectionType det_type      = (DetectionType) param<int>("detection type");
    SVMType       svm_type      = (SVMType) param<int>("svm type");

    switch(svm_type) {
    case DEFAULT:
        h.winSize = cv::Size(64, 128);
        h.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        break;
    case DAIMLER:
        h.winSize = cv::Size(48, 96);
        h.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
        break;
    case CUSTOM:
        setParameterEnabled("svm path", true);
        if(svm_.empty())
            return;
        h.setSVMDetector(svm_);
        break;
    default:
        throw std::runtime_error("Unkown SVM type!");
    }


    std::vector<cv::Rect>  loc_rects;
    std::vector<cv::Point> loc_points;
    switch(det_type) {
    case SINGLE_SCALE:
        h.detect(in->value, loc_points, det_threshold);
        break;
    case MULTI_SCALE:
        h.detectMultiScale(in->value, loc_rects, det_threshold);
        break;
    default:
        throw std::runtime_error("Unknown detection type!");
    }

    for(std::vector<cv::Rect>::iterator
        it = loc_rects.begin() ;
        it != loc_rects.end() ;
        ++it ) {
        cv::rectangle(out->value, *it, cv::Scalar(255,0,255), 1 , CV_AA);
    }

    std::cout << "count " << loc_rects.size() << std::endl;

    out_->publish(out);
}

void HOGDetect::load()
{
    std::string     path = param<std::string>("svm path");
    cv::FileStorage fs(path,cv::FileStorage::READ);

    if(!fs.isOpened()) {
        throw std::runtime_error("Couldn't open file '" + path + "'!");
    }

    svm_.clear();

    fs["svm"] >> svm_;

    if(svm_.empty())
        throw std::runtime_error("Couldn't load svm!");
    fs.release();

}

