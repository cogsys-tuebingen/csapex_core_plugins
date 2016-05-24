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

HOGDetector::HOGDetector() :
    prev_svm_type_(NONE),
    svm_type_(NONE)
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

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("svm/thresh", -10000.0, 10000.0, 0.0, 0.1),
                            svm_thresh_);

    std::function<bool()> custom_active = [svm_param]() { return svm_param->as<int>() == CUSTOM; };
    parameters.addConditionalParameter(param::ParameterFactory::declareFileInputPath("svm/path","", "*.yml *.yaml *.tar.gz"),
                                       custom_active, std::bind(&HOGDetector::load, this));

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
                                       signed_gradient_);

    parameters.addConditionalParameter(param::ParameterFactory::declareRange("hog/gradient_bins",
                                                                             param::ParameterDescription("Amount of gradient bins."),
                                                                             2, 18, hog_.nbins, 1),
                                       custom_active,
                                       n_bins_);


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
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("hog/block_size",
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
    out_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("detections");
}

void HOGDetector::process()
{
    CvMatMessage::ConstPtr  in = msg::getMessage<CvMatMessage>(in_);
    std::shared_ptr<std::vector<RoiMessage>> out(new std::vector<RoiMessage>);

    switch(svm_type_) {
    case DEFAULT:
        setParameters(8, 8, 16, 2, 1, 9, false);
        if(prev_svm_type_ != DEFAULT) {
            hog_.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        }
        prev_svm_type_ = DEFAULT;
        break;
    case DAIMLER:
        setParameters(8, 6, 12, 2, 1, 9, false);
        if(prev_svm_type_ != DAIMLER)
            hog_.setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
        prev_svm_type_ = DAIMLER;
        break;
    case CUSTOM:
        if(svm_.empty())
            return;
        setParameters(cell_size_, cells_x_, cells_y_, block_size_, block_stride_, n_bins_, signed_gradient_);
        if(prev_svm_type_ != CUSTOM)
            hog_.setSVMDetector(svm_);
        prev_svm_type_ = CUSTOM;
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
        RoiMessage roi;
        cv::Scalar color(utils_vision::color::bezierColor<cv::Scalar>(i / (float) loc_rects.size()));
        roi.value = Roi(loc_rects.at(i), color, HUMAN);
        out->push_back(roi);
    }

    for(unsigned int i = 0 ; i < loc_points.size() ; ++i) {
        RoiMessage roi;
        cv::Scalar color(utils_vision::color::bezierColor<cv::Scalar>(i / (float) loc_points.size()));
        cv::Point &p = loc_points.at(i);
        cv::Rect r(p.x, p.y, hog_.winSize.width, hog_.winSize.height);
        roi.value = Roi(r, color, HUMAN);
        out->push_back(roi);
    }
    msg::publish<GenericVectorMessage, RoiMessage>(out_, out);
}

void HOGDetector::load()
{
    std::string     path = readParameter<std::string>("svm/path");

    if(path == "")
        return;

    cv::FileStorage fs(path,cv::FileStorage::READ);

    if(!fs.isOpened()) {
        throw std::runtime_error("Couldn't open file '" + path + "'!");
    }

    svm_.clear();
    fs["svm_coeffs"] >> svm_;
    double rho;
    fs["svm_rho"] >> rho;
    setParameter<double>("svm/thresh", rho);

    if(svm_.empty())
        throw std::runtime_error("Couldn't load svm!");
    fs.release();

    hog_.setSVMDetector(svm_);
}

void HOGDetector::setParameters(const int cell_size,
                                const int cells_x, const int cells_y,
                                const int block_size,
                                const int block_stride,
                                const int bins,
                                const bool signed_gradient)
{
    if(hog_.cellSize.width != cell_size || hog_.cellSize.height != cell_size) {
        setParameter<int>("hog/cell_size", cell_size);
        hog_.cellSize.width  = cell_size;
        hog_.cellSize.height = cell_size;
    }

    int window_height = cell_size * cells_y;
    if(hog_.winSize.height != window_height) {
        setParameter<int>("hog/cells_y", cells_y);
        hog_.winSize.height = window_height;
    }

    int window_width = cell_size * cells_x;
    if(hog_.winSize.width !=  window_width) {
        hog_.winSize.width = window_width;
        setParameter<int>("hog/cells_x", cells_x);
    }

    int block_size_px = cell_size * block_size;
    if(hog_.blockSize.width != block_size_px ||
            hog_.blockSize.height != block_size_px) {
        hog_.blockSize.width = block_size_px;
        hog_.blockSize.height = block_size_px;
        setParameter<int>("hog/block_size", block_size);
    }

    int block_stride_px = cell_size * block_stride;
    if(hog_.blockStride.width != block_stride_px ||
            hog_.blockStride.height != block_stride_px) {
        hog_.blockStride.height = block_stride_px;
        hog_.blockStride.width = block_stride_px;
        setParameter<int>("hog/bock_stride", block_stride);
    }


    if(hog_.nbins != bins) {
        hog_.nbins = bins;
        setParameter<int>("hog/gradient_bins", bins);
    }

    if(hog_.signedGradient |= signed_gradient) {
        hog_.signedGradient = signed_gradient;
        setParameter<bool>("hog/signed_gradient", signed_gradient);
    }
}

