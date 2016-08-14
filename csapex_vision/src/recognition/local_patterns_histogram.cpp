/// HEADER
#include "local_patterns_histogram.h"

/// PROJECT
#include <csapex/msg/io.h>

#include <csapex_vision/cv_mat_message.h>
#include <csapex_ml/features_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_vision/roi_message.h>

#include <csapex/model/token_data.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>

#include <cslibs_vision/textures/lbp.hpp>
#include <cslibs_vision/textures/ltp.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::LocalPatternsHistogram, csapex::Node)

using namespace csapex;
using namespace vision_plugins;
using namespace connection_types;

namespace {
inline void lbp(const cv::Mat &src,
                std::vector<float> &result)
{
    std::vector<int> histogram;

    switch(src.type()) {
    case CV_8UC1:
         cslibs_vision::LBP::histogram<uchar>(src, 0, histogram);
         break;
    case CV_8SC1:
         cslibs_vision::LBP::histogram<char>(src, 0, histogram);
         break;
    case CV_16UC1:
         cslibs_vision::LBP::histogram<ushort>(src, 0, histogram);
         break;
    case CV_16SC1:
         cslibs_vision::LBP::histogram<short>(src, 0, histogram);
         break;
    case CV_32SC1:
         cslibs_vision::LBP::histogram<int>(src, 0, histogram);
         break;
    case CV_32FC1:
         cslibs_vision::LBP::histogram<float>(src, 0, histogram);
         break;
    case CV_64FC1:
         cslibs_vision::LBP::histogram<double>(src, 0, histogram);
         break;
    default:
        throw std::runtime_error("Unknown matrix type!");
    }

    result.assign(histogram.begin(), histogram.end());

}

inline void ltp(const cv::Mat &src,
                const double   k,
                std::vector<float> &result)
{
    std::vector<int> histogram;
    switch(src.type()) {
    case CV_8UC1:
         cslibs_vision::LTP::histogram<uchar>(src, k, histogram);
         break;
    case CV_8SC1:
         cslibs_vision::LTP::histogram<char>(src, k, histogram);
         break;
    case CV_16UC1:
         cslibs_vision::LTP::histogram<ushort>(src, k, histogram);
         break;
    case CV_16SC1:
         cslibs_vision::LTP::histogram<short>(src, k, histogram);
         break;
    case CV_32SC1:
         cslibs_vision::LTP::histogram<int>(src, k, histogram);
         break;
    case CV_32FC1:
         cslibs_vision::LTP::histogram<float>(src, k, histogram);
         break;
    case CV_64FC1:
         cslibs_vision::LTP::histogram<double>(src, k, histogram);
         break;
    default:
        throw std::runtime_error("Unknown matrix type!");
    }

    result.assign(histogram.begin(), histogram.end());
}
}

LocalPatternsHistogram::LocalPatternsHistogram()
{
}

void LocalPatternsHistogram::process()
{
    CvMatMessage::ConstPtr  in = msg::getMessage<CvMatMessage>(in_img_);
    std::shared_ptr< std::vector<FeaturesMessage> > out(new std::vector<FeaturesMessage>);

    if(in->value.channels() > 1)
        throw std::runtime_error("Matrix must be one channel!");

    double k = readParameter<double>("k1");
    Type   t = (Type) readParameter<int>("pattern");


    const cv::Mat &value = in->value;

    if(!msg::hasMessage(in_rois_)) {
        FeaturesMessage feature_msg;

        feature_msg.classification = 0;

        if(t == LBP) {
            lbp(value, feature_msg.value);
        } else {
            ltp(value, k, feature_msg.value);
        }

        out->push_back(feature_msg);

    } else {
        std::shared_ptr< std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

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

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, out);
}

void LocalPatternsHistogram::setup(NodeModifier& node_modifier)
{
    in_img_     = node_modifier.addInput<CvMatMessage>("image");
    in_rois_    = node_modifier.addOptionalInput<GenericVectorMessage, RoiMessage>("rois");
    out_        = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("descriptors");
}

void LocalPatternsHistogram::setupParameters(Parameterizable& parameters)
{
    std::map<std::string, int> types = {
        {"LBP", LBP},
        {"LTP", LTP}
    };

    csapex::param::Parameter::Ptr type =
            csapex::param::ParameterFactory::declareParameterSet("pattern",
                                                         types,
                                                         (int) LBP);
    std::function<bool()> condition = [type]() { return type->as<int>() == LTP; };

    parameters.addParameter(type);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange("k1", -100.0, 100.0, 0.0, 0.1),
                            condition);

}

