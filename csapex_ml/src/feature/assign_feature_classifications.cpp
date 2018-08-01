/// HEADER
#include "assign_feature_classifications.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>

CSAPEX_REGISTER_CLASS(csapex::AssignFeatureClassifications, csapex::Node)

using namespace csapex;
using namespace connection_types;

AssignFeatureClassifications::AssignFeatureClassifications()
{

}

void AssignFeatureClassifications::setup(NodeModifier &node_modifier)
{
    in_features_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("Features");
    in_labels_   = node_modifier.addOptionalInput<GenericVectorMessage, int>("Classifications");
    out_         = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Labelled features");
}

void AssignFeatureClassifications::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::factory::declareRange("label", 0, 255, 0, 1),
                            label_);
}

void AssignFeatureClassifications::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> in_features =
            msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_features_);
    std::shared_ptr<std::vector<FeaturesMessage>> out_features = std::make_shared<std::vector<FeaturesMessage>>();

    if(msg::hasMessage(in_labels_)) {
        std::shared_ptr<std::vector<int> const> in_labels = msg::getMessage<GenericVectorMessage, int>(in_labels_);

        if(in_features->size() != in_labels->size())
            throw std::runtime_error("Label count (" + std::to_string(in_labels->size()) + ") != FeatureMsg count (" + std::to_string(in_features->size()) + ")!");

        for(std::size_t i = 0 ; i < in_features->size() ; ++i) {
            FeaturesMessage feature = in_features->at(i);
            apex_assert(feature.type == FeaturesMessage::Type::CLASSIFICATION);
            int label = (int) in_labels->at(i);
            feature.classification = label;
            feature.type = FeaturesMessage::Type::CLASSIFICATION;
            out_features->emplace_back(feature);
        }
    } else {
        for(FeaturesMessage feature : *in_features) {
            feature.classification = label_;
            feature.type = FeaturesMessage::Type::CLASSIFICATION;
            out_features->emplace_back(feature);
        }
    }
    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, out_features);
}
