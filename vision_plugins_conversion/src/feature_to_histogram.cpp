/// HEADER
#include "feature_to_histogram.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <vision_plugins_histograms/histogram_msg.h>
#include <csapex_ml/features_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::FeatureToHistogram, csapex::Node)

FeatureToHistogram::FeatureToHistogram()
{
}

void FeatureToHistogram::process()
{
    FeaturesMessage::ConstPtr  in;
    VectorMessage::ConstPtr    in_vector;
    HistogramMessage::Ptr hist(new HistogramMessage);

    if(in_->hasMessage()) {
        in = in_->getMessage<FeaturesMessage>();
        hist->value.ranges.push_back(utils_vision::histogram::Range(0, in->value.size()));
        hist->value.histograms.push_back(cv::Mat(in->value, true));
    }
    if(in_vector_->hasMessage()) {
        in_vector = in_vector_->getMessage<VectorMessage>();

        for(auto it = in_vector->value.begin() ;
            it != in_vector->value.end() ;
            ++it) {

            FeaturesMessage::Ptr feature_msg = boost::dynamic_pointer_cast<FeaturesMessage>(*it);
            hist->value.ranges.push_back(utils_vision::histogram::Range(0, feature_msg->value.size()));
            hist->value.histograms.push_back(cv::Mat(feature_msg->value, true));
        }
    }

    out_->publish(hist);
}

void FeatureToHistogram::setup()
{
    in_        = modifier_->addOptionalInput<FeaturesMessage>("feature");
    in_vector_ = modifier_->addOptionalInput<VectorMessage, FeaturesMessage>("features");
    out_       = modifier_->addOutput<HistogramMessage>("histograms");
}
