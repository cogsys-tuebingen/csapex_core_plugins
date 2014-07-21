/// HEADER
#include "hog_detector.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <utils_cv/color_functions.hpp>


/// SYSTEM
#include <opencv2/objdetect/objdetect.hpp>
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::HOGDetector, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

HOGDetector::HOGDetector()
{
    addTag(Tag::get("vision_plugins"));
}

void HOGDetector::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("thresh", -10.0, 10.0, 0.0, 0.1));
    std::map<std::string, int> det_types = boost::assign::map_list_of
            ("single scale", SINGLE_SCALE)
            ("multi scale", MULTI_SCALE);
    addParameter(param::ParameterFactory::declareParameterSet("detection type", det_types, (int) SINGLE_SCALE));

    std::map<std::string, int> svm_types = boost::assign::map_list_of
            ("default", DEFAULT)
            ("custom",  CUSTOM)
            ("daimler", DAIMLER);

    param::Parameter::Ptr svm_param = param::ParameterFactory::declareParameterSet("svm type", svm_types, (int) DEFAULT);
    addParameter(svm_param);


    boost::function<bool()> condition = (boost::bind(&param::Parameter::as<int>, svm_param.get()) == CUSTOM);

    addConditionalParameter(param::ParameterFactory::declareFileInputPath("svm path","", "*.yml *.yaml *.tar.gz"),
                            condition, boost::bind(&HOGDetector::load, this));

    setParameterEnabled("svm path", false);
}

void HOGDetector::setup()
{
    in_  = modifier_->addInput<CvMatMessage>("image");
    out_ = modifier_->addOutput<VectorMessage, RoiMessage>("detections");
}

void HOGDetector::process()
{
    CvMatMessage::Ptr  in = in_->getMessage<CvMatMessage>();
    VectorMessage::Ptr out(VectorMessage::make<RoiMessage>());


    if(in->getEncoding() != enc::mono)
        throw std::runtime_error("Need grayscale or bgr!");

    cv::HOGDescriptor h;

    double        det_threshold = param<double>("thresh");
    DetectionType det_type      = (DetectionType) param<int>("detection type");
    SVMType       svm_type      = (SVMType) param<int>("svm type");

    switch(svm_type) {
    case DEFAULT:
        h.winSize.width  = 64;
        h.winSize.height = 128;
        h.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        break;
    case DAIMLER:
        h.winSize.width  = 48;
        h.winSize.height = 96;
        h.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
        break;
    case CUSTOM:
        if(svm_.empty())
            return;
        h.winSize.width  = svm_width_;
        h.winSize.height = svm_height_;
        h.setSVMDetector(svm_);
        break;
    default:
        throw std::runtime_error("Unkown SVM type!");
    }

    /// TODO
    h.gammaCorrection ;
    h.nlevels;
    h.winSigma;

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

    for(unsigned int i = 0 ; i < loc_rects.size() ; ++i) {
        RoiMessage::Ptr roi(new RoiMessage);
        cv::Scalar color(utils_cv::color::bezierColor<cv::Scalar>(i / (float) loc_rects.size()));
        roi->value = Roi(loc_rects.at(i), color, 0);
        out->value.push_back(roi);
    }

    for(unsigned int i = 0 ; i < loc_points.size() ; ++i) {
        RoiMessage::Ptr roi(new RoiMessage);
        cv::Scalar color(utils_cv::color::bezierColor<cv::Scalar>(i / (float) loc_points.size()));
        cv::Point &p = loc_points.at(i);
        cv::Rect r(p.x, p.y, svm_width_, svm_height_);
        roi->value = Roi(r, color, 0);
        out->value.push_back(roi);
    }
     out_->publish(out);
}

void HOGDetector::load()
{
    std::string     path = param<std::string>("svm path");

    if(path == "")
        return;

    cv::FileStorage fs(path,cv::FileStorage::READ);

    if(!fs.isOpened()) {
        throw std::runtime_error("Couldn't open file '" + path + "'!");
    }

    svm_.clear();

    fs["width"]  >> svm_width_;
    fs["height"] >> svm_height_;
    fs["svm"]    >> svm_;

    if(svm_.empty())
        throw std::runtime_error("Couldn't load svm!");
    fs.release();

}

