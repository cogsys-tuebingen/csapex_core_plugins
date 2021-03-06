/// HEADER
#include "feature_to_histogram.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
#include <csapex_vision_histograms/histogram_msg.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::FeatureToHistogram, csapex::Node)

FeatureToHistogram::FeatureToHistogram()
{
}

void FeatureToHistogram::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addOptionalInput<FeaturesMessage>("feature");
    in_vector_ = node_modifier.addOptionalInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<HistogramMessage>("histograms");
}

void FeatureToHistogram::process()
{
    FeaturesMessage::ConstPtr in;
    std::shared_ptr<std::vector<FeaturesMessage> const> in_vector;
    HistogramMessage::Ptr hist(new HistogramMessage);

    if (msg::hasMessage(in_)) {
        in = msg::getMessage<FeaturesMessage>(in_);
        hist->value.ranges.push_back(cslibs_vision::histogram::Rangef(0, in->value.size()));
        hist->value.histograms.push_back(cv::Mat(in->value, true));
    }
    if (msg::hasMessage(in_vector_)) {
        in_vector = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_vector_);

        for (const FeaturesMessage& fm : *in_vector) {
            hist->value.ranges.push_back(cslibs_vision::histogram::Rangef(0, fm.value.size()));
            hist->value.histograms.push_back(cv::Mat(fm.value, true));
        }
    }

    msg::publish(out_, hist);
}
