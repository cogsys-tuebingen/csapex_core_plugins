/// HEADER
#include "acf_standard_extractor.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_ml/features_message.h>

using namespace csapex;
using namespace connection_types;


CSAPEX_REGISTER_CLASS(csapex::ACFStandardExtractor, csapex::Node)

ACFStandardExtractor::ACFStandardExtractor() :
    ratio_w_h_(0.0)
{
}

void ACFStandardExtractor::setupParameters(Parameterizable &parameters)
{
    // window setting
    parameters.addParameter(param::factory::declareRange("window/width",
                                                                  10, 1024, 64, 1),
                            std::bind(&ACFStandardExtractor::updateWindow, this));
    parameters.addParameter(param::factory::declareRange("window/height",
                                                                  10, 1024, 128, 1),
                            std::bind(&ACFStandardExtractor::updateWindow, this));
    parameters.addParameter(param::factory::declareBool("window/mirror",
                                                                 false),
                            mirror_);
    parameters.addParameter(param::factory::declareBool("window/keep_ratio",
                                                                 false),
                            keep_ratio_);

    // kernel
    static const std::map<std::string, int> kernel_types = {
            {"1D", cslibs_vision::ACF::Parameters::KERNEL_1D},
            {"2D", cslibs_vision::ACF::Parameters::KERNEL_2D},
            {"NONE", cslibs_vision::ACF::Parameters::NONE}
    };
    parameters.addParameter(param::factory::declareParameterSet("kernel_type",
                                                                         kernel_types,
                                                                         (int) cslibs_vision::ACF::Parameters::KERNEL_2D),
                            (int &) acf_params_.kernel_type);

    // HOG channel parameter
    parameters.addParameter(param::factory::declareRange("hog/bin_size",
                                                                  5.0, 90.0, 30.0, 0.1),
                            acf_params_.hog_bin_size);
    parameters.addParameter(param::factory::declareBool("hog/directed", false),
                            acf_params_.hog_directed);

    // HOG/magnitude channel parameter
    parameters.addParameter(param::factory::declareBool("magnitude/normalize", true),
                            acf_params_.normalize_magnitude);

    // LUV channel parameter
    parameters.addParameter(param::factory::declareBool("luv/normalize", true),
                            acf_params_.normalize_luv);

}

void ACFStandardExtractor::setup(NodeModifier &node_modifier)
{
    in_img_         = node_modifier.addInput<CvMatMessage>("Image");
    in_rois_        = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
    out_features_   = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Features");
}

void ACFStandardExtractor::process()
{
    CvMatMessage::ConstPtr in_img = msg::getMessage<CvMatMessage>(in_img_);
    std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    if(in_img->getEncoding().matches(enc::bgr) &&
            in_img->getEncoding().matches(enc::mono)) {
        throw std::runtime_error("Input image must be single channel grayscale or three channel bgr.");
    }

    cslibs_vision::ACFStandard::Parameters params = acf_params_;
    params.hog_bin_size = cslibs_vision::ACF::rad(params.hog_bin_size);

    std::shared_ptr<std::vector<FeaturesMessage>> out_features(new std::vector<FeaturesMessage>);
    for(const RoiMessage &roi : *in_rois) {
        cv::Mat roi_mat;
        cv::resize(cv::Mat(in_img->value, roi.value.rect()  & cv::Rect(0, 0, in_img->value.cols, in_img->value.rows)), roi_mat, window_size_);

        cv::Mat feature;
        cslibs_vision::ACFStandard::compute(roi_mat, params, feature);
        FeaturesMessage features_msg;
        features_msg.type = FeaturesMessage::Type::CLASSIFICATION;
        feature.copyTo(features_msg.value);
        features_msg.classification = roi.value.classification();
        out_features->emplace_back(features_msg);

        if (mirror_) {
            cv::flip(roi_mat, roi_mat, 1);

            cv::Mat feature;
            cslibs_vision::ACFStandard::compute(roi_mat, params, feature);
            FeaturesMessage features_msg;
            features_msg.type = FeaturesMessage::Type::CLASSIFICATION;
            feature.copyTo(features_msg.value);
            features_msg.classification = roi.value.classification();
            out_features->emplace_back(features_msg);
        }
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_features_, out_features);
}

void ACFStandardExtractor::updateWindow()
{
    window_size_.width  = readParameter<int>("window/width");
    window_size_.height = readParameter<int>("window/height");
    if(ratio_w_h_ == 0.0) {
        ratio_w_h_ = window_size_.height / (double) window_size_.width;
    }

    if(keep_ratio_) {
        if(window_size_.height != window_size_before_.height) {
            window_size_.width = window_size_.height / ratio_w_h_;
            setParameter<int>("window/width", window_size_.width);
        } else {
            window_size_.height = window_size_.width * ratio_w_h_;
            setParameter<int>("window/height", window_size_.height);
        }
    } else {
        window_size_.width  = readParameter<int>("window/width");
        window_size_.height = readParameter<int>("window/height");
    }

    if(window_size_before_ == cv::Size()) {
        window_size_before_ = window_size_;
    }

    window_size_before_ = window_size_;
}
