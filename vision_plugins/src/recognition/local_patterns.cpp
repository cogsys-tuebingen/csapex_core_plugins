/// HEADER
#include "local_patterns.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

#include <csapex_vision/cv_mat_message.h>
#include <csapex_ml/features_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/roi_message.h>

#include <csapex/model/connection_type.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>

#include <utils_vision/utils/local_patterns.hpp>

/// SYSTEM
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::LocalPatterns, csapex::Node)

using namespace csapex;
using namespace vision_plugins;
using namespace connection_types;

namespace {
inline void lbp(const cv::Mat &src,
                std::vector<float> &result)
{
    utils_vision::LBP pattern;
    switch(src.type()) {
    case CV_8UC1:
        pattern.stdExtraction<uchar>(src);
        break;
    case CV_8SC1:
        pattern.stdExtraction<char>(src);
        break;
    case CV_16UC1:
        pattern.stdExtraction<ushort>(src);
        break;
    case CV_16SC1:
        pattern.stdExtraction<short>(src);
        break;
    case CV_32SC1:
        pattern.stdExtraction<int>(src);
        break;
    case CV_32FC1:
        pattern.stdExtraction<float>(src);
        break;
    case CV_64FC1:
        pattern.stdExtraction<double>(src);
        break;
    }

    std::vector<int> histogram;
    pattern.getHistogram(histogram);

    float max = *(std::max_element(histogram.begin(), histogram.end()));
    result.resize(histogram.size());
    for(unsigned int i = 0 ; i < histogram.size() ; ++i) {
        result.at(i) = histogram.at(i) / max;
    }

}

inline void ltp(const cv::Mat &src,
                const double   k,
                std::vector<float> &result)
{
    utils_vision::LTP pattern;
    switch(src.type()) {
    case CV_8UC1:
        pattern.stdExtraction<uchar>(src, k);
        break;
    case CV_8SC1:
        pattern.stdExtraction<char>(src, k);
        break;
    case CV_16UC1:
        pattern.stdExtraction<ushort>(src, k);
        break;
    case CV_16SC1:
        pattern.stdExtraction<short>(src, k);
        break;
    case CV_32SC1:
        pattern.stdExtraction<int>(src, k);
        break;
    case CV_32FC1:
        pattern.stdExtraction<float>(src, k);
        break;
    case CV_64FC1:
        pattern.stdExtraction<double>(src, k);
        break;
    }

    std::vector<int> histogram;
    pattern.getAll(histogram);

    float max = *(std::max_element(histogram.begin(), histogram.end()));
    result.resize(histogram.size());
    for(unsigned int i = 0 ; i < histogram.size() ; ++i) {
        result.at(i) = histogram.at(i) / max;
    }
}
}

LocalPatterns::LocalPatterns()
{
}

void LocalPatterns::process()
{
    CvMatMessage::ConstPtr  in = in_img_->getMessage<CvMatMessage>();
    std::shared_ptr< std::vector<FeaturesMessage> > out(new std::vector<FeaturesMessage>);

    if(in->value.channels() > 1)
        throw std::runtime_error("Matrix must be one channel!");

    double k = readParameter<double>("k1");
    Type   t = (Type) readParameter<int>("pattern");


    const cv::Mat &value = in->value;

    if(!in_rois_->hasMessage()) {
        FeaturesMessage feature_msg;

        feature_msg.classification = 0;

        if(t == LBP) {
            lbp(value, feature_msg.value);
        } else {
            ltp(value, k, feature_msg.value);
        }

        out->push_back(feature_msg);

    } else {
        std::shared_ptr< std::vector<RoiMessage> const> in_rois = in_rois_->getMessage<GenericVectorMessage, RoiMessage>();

        for(std::vector<RoiMessage>::const_iterator
            it  = in_rois->begin() ;
            it != in_rois->end() ;
            ++it) {

            FeaturesMessage feature_msg;
            cv::Rect const &rect = it->value.rect();

            cv::Mat roi_mat = cv::Mat(value, rect);

            if(t == LBP) {
                lbp(roi_mat, feature_msg.value);
            } else {
                ltp(roi_mat, k, feature_msg.value);
            }

            feature_msg.classification = it->value.classification();

            out->push_back(feature_msg);
        }
    }

    out_->publish<GenericVectorMessage, FeaturesMessage>(out);
}

void LocalPatterns::setup()
{
    in_img_     = modifier_->addInput<CvMatMessage>("image");
    in_rois_    = modifier_->addOptionalInput<GenericVectorMessage, RoiMessage>("rois");
    out_        = modifier_->addOutput<GenericVectorMessage, FeaturesMessage>("descriptors");
}

void LocalPatterns::setupParameters()
{
    std::map<std::string, int> types =
            boost::assign::map_list_of
            ("LBP", LBP)("LTP", LTP);

    param::Parameter::Ptr type =
            param::ParameterFactory::declareParameterSet("pattern",
                                                         types,
                                                         (int) LBP);
    std::function<bool()> condition = [type]() { return type->as<int>() == LTP; };

    addParameter(type);
    addConditionalParameter(param::ParameterFactory::declareRange("k1", -100.0, 100.0, 0.0, 0.1),
                            condition);

}

