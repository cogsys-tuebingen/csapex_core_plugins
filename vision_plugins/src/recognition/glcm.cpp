/// HEADER
#include "glcm.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>

CSAPEX_REGISTER_CLASS(vision_plugins::GLCM, csapex::Node)

using namespace csapex;
using namespace vision_plugins;
using namespace connection_types;


namespace {
inline void glcm4(const cv::Mat &src, const unsigned int bins, cv::Mat &dst)
{
    assert(bins > 0);
    const unsigned int max_idx = src.cols - 1;
    const unsigned int max_idy = src.rows - 1;

    dst = cv::Mat(bins, bins, CV_32S, cv::Scalar::all(0));

    const float max      = 255.0f;
    const float bins_max = (float)(bins - 1);
    const float iscale   = (1.0f / max) * bins_max;

    for(unsigned int i = 0 ; i < max_idy; ++i) {
        for(unsigned int j = 0 ; j < max_idx; ++j) {
            const float c = (float)src.at<uchar>(i,j);
            const float r = (float)src.at<uchar>(i,j + 1);
            const float b = (float)src.at<uchar>(i + 1,j);

            const int bin_c = std::floor((c * iscale) + 0.5f);
            const int bin_r = std::floor((r * iscale) + 0.5f);
            const int bin_b = std::floor((b * iscale) + 0.5f);

            dst.at<int>(bin_c, bin_r) += 1;    dst.at<int>(bin_r, bin_c) += 1;
            dst.at<int>(bin_c, bin_b) += 1;    dst.at<int>(bin_b, bin_c) += 1;
        }
        float c = (float)src.at<uchar>(i,     max_idx);
        float b = (float)src.at<uchar>(i + 1, max_idx);

        const int bin_c = std::floor((c * iscale) + 0.5f);
        const int bin_b = std::floor((b * iscale) + 0.5f);

        dst.at<int>(bin_c, bin_b) += 1;    dst.at<int>(bin_b, bin_c) += 1;

    }
    for(unsigned int j = 0 ; j < max_idx; ++j) {
        float c = (float)src.at<uchar>(max_idy, j);
        float r = (float)src.at<uchar>(max_idy, j + 1);

        const int bin_c = std::floor((c * iscale) + 0.5f);
        const int bin_r = std::floor((r * iscale) + 0.5f);

        dst.at<int>(bin_c, bin_r) += 1;    dst.at<int>(bin_r, bin_c) += 1;
    }
}

inline void glcm8(const cv::Mat &src, const unsigned int bins, cv::Mat &dst)
{
    throw std::runtime_error("not implemented");
}
}


GLCM::GLCM()
{
}

void GLCM::process()
{
    CvMatMessage::Ptr in = in_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr out(new CvMatMessage(enc::mono, in->stamp));

    if(in->value.type() != CV_8UC1)
        std::runtime_error("Only mono 8 supported!");

    unsigned int bins = readParameter<int>("bins");
    glcm4(in->value, bins, out->value);

    out_->publish(out);
}

void GLCM::setup()
{
   in_  = modifier_->addInput<CvMatMessage>("mono image");
   out_ = modifier_->addOutput<CvMatMessage>("glcm");
}

void GLCM::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("bins",
                                                       param::ParameterDescription("GLCM histogram bins."),
                                                       2, 255, 10, 1));
}
