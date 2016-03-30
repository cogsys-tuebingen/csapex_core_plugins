/// HEADER
#include "hog_classifier.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_ml/features_message.h>

/// SYSTEM
#include <boost/assign.hpp>
#include <opencv2/objdetect/objdetect.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::HOGClassifier, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

HOGClassifier::HOGClassifier() :
    prev_svm_type_(NONE)
{
}

void HOGClassifier::setupParameters(Parameterizable& parameters)
{

    std::map<std::string, int> adpation_types =
            boost::assign::map_list_of("Scale", SCALE)("TryGrow", TRY_GROW)("GrowStrict", GROW_STRICT);
    parameters.addParameter(param::ParameterFactory::declareParameterSet("hog/adaption_mode",
                                                                         param::ParameterDescription("Adaption of rois to window size of hog."),
                                                                         adpation_types,
                                                                         (int) SCALE),
                            adaption_type_);

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
                                       custom_active, std::bind(&HOGClassifier::load, this));


    parameters.addParameter(param::ParameterFactory::declareRange("svm/thresh", -10.0, 10.0, 0.0, 0.1),
                            svm_thresh_);

    parameters.addParameter(param::ParameterFactory::declareBool("hog/mirror",
                                                                 param::ParameterDescription("Mirror for classification."),
                                                                 false),
                            mirror_);

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

void HOGClassifier::setup(NodeModifier& node_modifier)
{
    in_img_     = node_modifier.addInput<CvMatMessage>("image");
    in_rois_    = node_modifier.addInput<VectorMessage, RoiMessage>("rois");
    out_rois_   = node_modifier.addOutput<VectorMessage, RoiMessage>("filtered rois");
}

void HOGClassifier::process()
{
    CvMatMessage::ConstPtr  in = msg::getMessage<CvMatMessage>(in_img_);
    VectorMessage::ConstPtr in_rois = msg::getMessage<VectorMessage>(in_rois_);
    VectorMessage::Ptr      out(VectorMessage::make<RoiMessage>());

    if(in->value.channels() != 1 && in->value.channels() != 3) {
        throw std::runtime_error("Only 1 or 3 channel matrices supported!");
    }

    /// update HOG parameters
    ratio_hog_ = hog_.winSize.width / (double) hog_.winSize.height;

    switch(svm_type_) {
    case DEFAULT:
        setParameters(8, 8, 16, 2, 1, 9, false);
        if(prev_svm_type_ != DEFAULT)
            hog_.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
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

    for(auto &entry : in_rois->value) {
        RoiMessage::ConstPtr roi  = std::dynamic_pointer_cast<RoiMessage const>(entry);
        cv::Mat data;

        getData(in->value, roi->value.rect(), data);

        if(data.empty())
            continue;

        assert(data.rows == hog_.winSize.height);
        assert(data.cols == hog_.winSize.width);

        double weight = 0.0;
        if(mirror_) {
            cv::Mat data_mirrored;
            cv::flip(data, data_mirrored, 1);
            bool original = hog_.classify(data, svm_thresh_, weight);
            bool mirrored = hog_.classify(data_mirrored, svm_thresh_, weight);
            if(!original && !mirrored) {
                continue;
            }
        } else {
            if(!hog_.classify(data, svm_thresh_, weight)) {
                continue;
            }
        }

        RoiMessage::Ptr roi_out(new RoiMessage);
        roi_out->value = roi->value;
        roi_out->value.setClassification(HUMAN);
        out->value.push_back(roi_out);
    }
    msg::publish(out_rois_, out);
}

void HOGClassifier::getData(const cv::Mat &src, const cv::Rect &roi, cv::Mat &dst)
{
    cv::Mat window;
    double ratio_roi = roi.width / roi.height;
    cv::Rect roi_adapted = roi;

    switch(adaption_type_) {
    case SCALE:
        break;
    case TRY_GROW:
    case GROW_STRICT:
        if(ratio_roi < ratio_hog_) {
            /// scale the width
            double scale_width = ratio_roi / ratio_hog_;
            roi_adapted.width *= scale_width;
            roi_adapted.x -= (roi_adapted.width - roi.width) / 2;
            /// move to fit into image
            if(roi_adapted.x < 0) {
                roi_adapted.x = 0;
            } else {
                int overshoot = (src.cols - 1) - (roi_adapted.x + roi.width);
                if(overshoot < 0) {
                    roi_adapted.x += overshoot;
                }
            }
        } else if(ratio_roi > ratio_hog_) {
            /// scale the height
            double scale_height = ratio_hog_ / ratio_roi;
            roi_adapted.height *= scale_height;
            roi_adapted.y -= (roi_adapted.height - roi.height) / 2;
            /// move to fit into image
            if(roi_adapted.y < 0) {
                roi_adapted.y = 0;
            } else {
                int overshoot = (src.rows - 1) - (roi_adapted.y + roi.height);
                if(overshoot < 0) {
                    roi_adapted.y += overshoot;
                }
            }
        }
        if(roi.height > src.rows || roi.width > src.cols) {
            cv::Rect img_rect(0,0,src.cols, src.rows);
            roi_adapted = roi_adapted & img_rect;

            if(adaption_type_ == GROW_STRICT)
                return;
        }
        break;
    default:
        throw std::runtime_error("Unknown adaption type!");
    }

    window = cv::Mat(src, roi_adapted);
    cv::resize(window, dst, hog_.winSize);
}

void HOGClassifier::load()
{
    std::string     path = readParameter<std::string>("svm/path");

    if(path == "")
        return;

    cv::FileStorage fs(path,cv::FileStorage::READ);

    if(!fs.isOpened()) {
        throw std::runtime_error("Couldn't open file '" + path + "'!");
    }

    svm_.clear();
    fs["svm_coeffs"]    >> svm_;
    double rho;
    fs["svm_rho"] >> rho;
    svm_.push_back(-rho);
    //    setParameter<double>("svm/thresh", rho);

    if(svm_.empty())
        throw std::runtime_error("Couldn't load svm!");
    fs.release();
}


void HOGClassifier::setParameters(const int cell_size,
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
