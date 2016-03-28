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
/// TODO : L2HysThreshold - derivAperture

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
    std::map<std::string, int> svm_types = {
        {"default", DEFAULT},
        {"custom",  CUSTOM},
        {"daimler", DAIMLER}
    };
    param::Parameter::Ptr svm_param =
            param::ParameterFactory::declareParameterSet("svm/type", svm_types, (int) DEFAULT);
    parameters.addParameter(svm_param,
                            svm_type_);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("svm/thresh", -10.0, 10.0, 0.0, 0.1),
                            svm_thresh_);

    std::function<bool()> custom_active = [svm_param]() { return svm_param->as<int>() == CUSTOM; };
    parameters.addConditionalParameter(param::ParameterFactory::declareFileInputPath("svm/path","", "*.yml *.yaml *.tar.gz"),
                                       custom_active, std::bind(&HOGDetector::load, this));
    setParameterEnabled("svm/path", false);

    /// scan mode
    std::map<std::string, int> scan_modes = {
        {"single scale", SINGLE_SCALE},
        {"multi scale", MULTI_SCALE}
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("hog/scan_mode", scan_modes, (int) SINGLE_SCALE),
                            scan_mode_);

    parameters.addParameter(param::ParameterFactory::declareRange("hog/levels",
                                                                   param::ParameterDescription("Scale levels to observe."),
                                                                   1, 128, 64, 1),
                 hog_.nlevels);

    parameters.addParameter(param::ParameterFactory::declareRange("hog/sigma",
                                                       param::ParameterDescription("Standard deviation for Gaussian blur."),
                                                       0.0, 10.0, 0.0, 0.1),
                 hog_.winSigma);

    parameters.addParameter(param::ParameterFactory::declareBool("hog/gamma_correction",
                                                      param::ParameterDescription("Enable the gamma correction."),
                                                      true),
                 hog_.gammaCorrection);


    /// paramters only applicable if custom mode is active
    parameters.addConditionalParameter(param::ParameterFactory::declareBool("hog/signed_gradient",
                                                                            param::ParameterDescription("Un-/directed gradients."),
                                                                            hog_.signedGradient),
                                       custom_active,
                                       hog_.signedGradient);

    parameters.addConditionalParameter(param::ParameterFactory::declareRange("hog/gradient_bins",
                                                                             param::ParameterDescription("Amount of gradient bins."),
                                                                             2, 18, hog_.nbins, 1),
                                       custom_active,
                                       hog_.nbins);


    parameters.addConditionalParameter(param::ParameterFactory::declareRange("hog/cells_x",
                                                                  param::ParameterDescription("Cells in x direction."),
                                                                  2, 16, 8, 1),
                                       custom_active,
                                       cells_x_);

    parameters.addConditionalParameter(param::ParameterFactory::declareRange("hog/cells_y",
                                                                              param::ParameterDescription("Cells in x direction."),
                                                                              2, 16, 16, 1),
                                       custom_active,
                                       cells_y_);

    parameters.addConditionalParameter(param::ParameterFactory::declareRange("hog/cell_size",
                                                                             param::ParameterDescription("Size of the cells."),
                                                                             4, 16, 8, 1),
                                       custom_active,
                                       cell_size_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("hog/block size",
                                                                              param::ParameterDescription("Cell count in both dimension of a block."),
                                                                              1, 4, 2, 1),
                                       custom_active,
                                       block_size_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("hog/bock_stride",
                                                                              param::ParameterDescription("Overlap of each block in cells."),
                                                                              1, 3, 1, 1),
                                       custom_active,
                                       block_stride_);
}

void HOGDetector::setup(NodeModifier& node_modifier)
{
    in_  = node_modifier.addInput<CvMatMessage>("image");
    out_ = node_modifier.addOutput<VectorMessage, RoiMessage>("detections");
}

void HOGDetector::process()
{
    CvMatMessage::ConstPtr  in = msg::getMessage<CvMatMessage>(in_);
    VectorMessage::Ptr      out(VectorMessage::make<RoiMessage>());
//    HOGDescriptor() : winSize(64,128), blockSize(16,16), blockStride(8,8),
//        cellSize(8,8), nbins(9), derivAperture(1), winSigma(-1),
//        histogramNormType(HOGDescriptor::L2Hys), L2HysThreshold(0.2), gammaCorrection(true),
//        free_coef(-1.f), nlevels(HOGDescriptor::DEFAULT_NLEVELS), signedGradient(false)
//    {}

    switch(svm_type_) {
    case DEFAULT:
        hog_.winSize.width  = 64;
        hog_.winSize.height = 128;
        hog_.signedGradient = false;
        hog_.blockSize = cv::Size(16,16);
        hog_.blockStride = cv::Size(8,8);
        hog_.cellSize = cv::Size(8,8);
        hog_.nbins = 9;
        hog_.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        break;
    case DAIMLER:
        hog_.winSize.width  = 48;
        hog_.winSize.height = 96;
        hog_.signedGradient = false;
        hog_.blockSize = cv::Size(16,16);
        hog_.blockStride = cv::Size(8,8);
        hog_.cellSize = cv::Size(8,8);
        hog_.nbins = 9;
        hog_.setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
        break;
    case CUSTOM:
        if(svm_.empty())
            return;
        hog_.cellSize.width  = cell_size_;
        hog_.cellSize.height = cell_size_;
        hog_.winSize.height  = cell_size_ * cells_y_;
        hog_.winSize.width   = cell_size_ * cells_x_;
        hog_.blockSize       = hog_.cellSize * block_size_;
        hog_.blockStride     = hog_.cellSize * block_stride_;
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
        roi->value = Roi(loc_rects.at(i), color, HUMAN);
        out->value.push_back(roi);
    }

    for(unsigned int i = 0 ; i < loc_points.size() ; ++i) {
        RoiMessage::Ptr roi(new RoiMessage);
        cv::Scalar color(utils_vision::color::bezierColor<cv::Scalar>(i / (float) loc_points.size()));
        cv::Point &p = loc_points.at(i);
        cv::Rect r(p.x, p.y, hog_.winSize.width, hog_.winSize.height);
        roi->value = Roi(r, color, HUMAN);
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
    fs["svm_coeffs"]    >> svm_;

    if(svm_.empty())
        throw std::runtime_error("Couldn't load svm!");
    fs.release();

}

