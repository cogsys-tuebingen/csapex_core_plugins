/// HEADER
#include "evaluate_feature_classification.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/signal/slot.h>

CSAPEX_REGISTER_CLASS(csapex::EvaluateFeatureClassification, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

EvaluateFeatureClassification::BinaryClassificationMetrics::BinaryClassificationMetrics(BinaryClassificationResult result):
    classification(result),

    tpr(result.tp / double(result.p)),
    tnr(result.tn / double(result.n)),
    ppv(result.tp / double(result.tp + result.fp)),
    npv(result.tn / double(result.tn + result.fn)),
    fpr(result.fp / double(result.n)),
    fdr(1 - ppv),
    fnr(result.fn / double(result.p)),
    acc((result.tp + result.tn) / double(result.p + result.n)),
    f1s(2 * result.tp / double(2 * result.tp + result.fp + result.fn)),
    mcc((result.tp * result.tn - result.fp * result.fn) /
        std::sqrt(double(result.tp + result.fp) * double(result.tp + result.fn) *
                  double(result.tn + result.fp) * double(result.tn + result.fn)))
{

}


EvaluateFeatureClassification::EvaluateFeatureClassification()
{
}

void EvaluateFeatureClassification::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareTrigger("reset"), [&](csapex::param::Parameter*) { confusion_.reset(); });
}

void EvaluateFeatureClassification::setup(NodeModifier& node_modifier)
{
    in_truth_  = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("True feature");
    in_classified_  = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Classified feature");

    out_ = node_modifier.addOutput<ConfusionMatrixMessage>("Evaluation Result");

    confusion_ = ConfusionMatrix();
}

void EvaluateFeatureClassification::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> truth_msg = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_truth_);
    std::shared_ptr<std::vector<FeaturesMessage> const> classified_msg = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_classified_);

    std::size_t n = truth_msg->size();

    for(std::size_t i = 0; i < n; ++i) {
        const FeaturesMessage& truth = truth_msg->at(i);
        const FeaturesMessage& classified = classified_msg->at(i);

        apex_assert(truth.type == FeaturesMessage::Type::CLASSIFICATION);
        apex_assert(classified.type == FeaturesMessage::Type::CLASSIFICATION);

        apex_assert(truth.value.size() == classified.value.size());

        const float& t = truth.classification;
        const float& c = classified.classification;

        confusion_.reportClassification(t, c);
    }

    ConfusionMatrixMessage::Ptr result(new ConfusionMatrixMessage);
    result->confusion = confusion_;
    msg::publish(out_, result);
}
