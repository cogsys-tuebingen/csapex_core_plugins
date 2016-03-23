/// HEADER
#include "hog_detector.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <utils_vision/utils/color_functions.hpp>

/// https://github.com/DaHoC/trainHOG/wiki/trainHOG-Tutorial

/// SYSTEM
CSAPEX_REGISTER_CLASS(vision_plugins::HOGDetector, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

HOGDetector::HOGDetector()
{
}

void HOGDetector::setupParameters(Parameterizable& parameters)
{
    /// scan mode
    std::map<std::string, int> scan_modes = {
        {"single scale", SINGLE_SCALE},
        {"multi scale", MULTI_SCALE}
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("hog/scan_mode", scan_modes, (int) SINGLE_SCALE),
                            scan_mode_);

    addParameter(param::ParameterFactory::declareRange("hog/levels",
                                                       param::ParameterDescription("Scale levels to observe."),
                                                       1, 128, 64, 1),
                 hog_.nlevels);

    addParameter(param::ParameterFactory::declareRange("hog/sigma",
                                                       param::ParameterDescription("Standard deviation for Gaussian blur."),
                                                       0.0, 10.0, 0.0, 0.1),
                 hog_.winSigma);

    addParameter(param::ParameterFactory::declareBool("hog/gamma_correction",
                                                      param::ParameterDescription("Enable the gamma correction."),
                                                      true),
                 hog_.gammaCorrection);
    addParameter(param::ParameterFactory::declareBool("hog/signed_gradient",
                                                      param::ParameterDescription("Un-/directed gradients."),
                                                      hog_.signedGradient),
                 hog_.signedGradient);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("svm/thresh", -10.0, 10.0, 0.0, 0.1),
                            svm_thresh_);

    std::map<std::string, int> svm_types = {
        {"default", DEFAULT},
        {"custom",  CUSTOM},
        {"daimler", DAIMLER}
    };
    param::Parameter::Ptr svm_param =
            param::ParameterFactory::declareParameterSet("svm/type", svm_types, (int) DEFAULT);
    parameters.addParameter(svm_param,
                            svm_type_);


    std::function<bool()> custom_active = [svm_param]() { return svm_param->as<int>() == CUSTOM; };
    parameters.addConditionalParameter(param::ParameterFactory::declareFileInputPath("svm/path","", "*.yml *.yaml *.tar.gz"),
                                       custom_active, std::bind(&HOGDetector::load, this));
    setParameterEnabled("svm/path", false);

}

void HOGDetector::setup(NodeModifier& node_modifier)
{
    in_  = node_modifier.addInput<CvMatMessage>("image");
    out_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("detections");
}

void HOGDetector::process()
{
    CvMatMessage::ConstPtr  in = msg::getMessage<CvMatMessage>(in_);
    VectorMessage::Ptr      out(VectorMessage::make<RoiMessage>());

    switch(svm_type_) {
    case DEFAULT:
        hog_.winSize.width  = 64;
        hog_.winSize.height = 128;
        hog_.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        break;
    case DAIMLER:
        hog_.winSize.width  = 48;
        hog_.winSize.height = 96;
        hog_.setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
        break;
    case CUSTOM:
        if(svm_.empty())
            return;
        hog_.winSize.width  = hog_win_width_;
        hog_.winSize.height = hog_win_height_;
        hog_.setSVMDetector(svm_);
        break;
    default:
        throw std::runtime_error("Unkown SVM type!");
    }

    if(hog_.winSigma == 0.0)
        hog_.winSigma = -1.0;

    std::vector<cv::Rect>  loc_rects;
    std::vector<cv::Point> loc_points;
    switch(scan_mode_) {
    case SINGLE_SCALE:
        hog_.detect(in->value, loc_points, svm_thresh_);
        break;
    case MULTI_SCALE:
        hog_.detectMultiScale(in->value, loc_rects, svm_thresh_);
        break;
    default:
        throw std::runtime_error("Unknown detection type!");
    }

    for(unsigned int i = 0 ; i < loc_rects.size() ; ++i) {
        RoiMessage::Ptr roi(new RoiMessage);
        cv::Scalar color(utils_vision::color::bezierColor<cv::Scalar>(i / (float) loc_rects.size()));
        roi->value = Roi(loc_rects.at(i), color, 0);
        out->value.push_back(roi);
    }

    for(unsigned int i = 0 ; i < loc_points.size() ; ++i) {
        RoiMessage::Ptr roi(new RoiMessage);
        cv::Scalar color(utils_vision::color::bezierColor<cv::Scalar>(i / (float) loc_points.size()));
        cv::Point &p = loc_points.at(i);
        cv::Rect r(p.x, p.y, hog_win_width_, hog_win_height_);
        roi->value = Roi(r, color, 0);
        out->value.push_back(roi);
    }
    msg::publish(out_, out);
}

void HOGDetector::load()
{
    std::string     path = readParameter<std::string>("svm path");

    if(path == "")
        return;

    cv::FileStorage fs(path,cv::FileStorage::READ);

    if(!fs.isOpened()) {
        throw std::runtime_error("Couldn't open file '" + path + "'!");
    }

    svm_.clear();

    fs["width"]  >> hog_win_width_;
    fs["height"] >> hog_win_height_;
    fs["svm"]    >> svm_;

    if(svm_.empty())
        throw std::runtime_error("Couldn't load svm!");
    fs.release();

}

