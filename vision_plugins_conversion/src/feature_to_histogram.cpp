/// HEADER
#include "feature_to_histogram.h"

/// PROJECT
#include <csapex/msg/io.h>
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

    if(msg::hasMessage(in_)) {
        in = msg::getMessage<FeaturesMessage>(in_);
        hist->value.ranges.push_back(utils_vision::histogram::Rangef(0, in->value.size()));
        hist->value.histograms.push_back(cv::Mat(in->value, true));
    }
    if(msg::hasMessage(in_vector_)) {
        in_vector = msg::getMessage<VectorMessage>(in_vector_);

        for(auto it = in_vector->value.begin() ;
            it != in_vector->value.end() ;
            ++it) {

            FeaturesMessage::ConstPtr feature_msg = std::dynamic_pointer_cast<FeaturesMessage const>(*it);
            hist->value.ranges.push_back(utils_vision::histogram::Rangef(0, feature_msg->value.size()));
            hist->value.histograms.push_back(cv::Mat(feature_msg->value, true));
        }
    }

    msg::publish(out_, hist);
}

void FeatureToHistogram::setup(NodeModifier& node_modifier)
{
    in_        = node_modifier.addOptionalInput<FeaturesMessage>("feature");
    in_vector_ = node_modifier.addOptionalInput<VectorMessage, FeaturesMessage>("features");
    out_       = node_modifier.addOutput<HistogramMessage>("histograms");
}
