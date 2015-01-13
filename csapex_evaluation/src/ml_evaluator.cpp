/// HEADER
#include "ml_evaluator.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/signal/slot.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::MLEvaluator, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

MLEvaluator::BinaryClassificationMetrics::BinaryClassificationMetrics(BinaryClassificationResult result):
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


MLEvaluator::MLEvaluator()
{
}

void MLEvaluator::setupParameters()
{
}

void MLEvaluator::setup()
{
    in_truth_  = modifier_->addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("True feature");
    in_classified_  = modifier_->addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Classified feature");

    out_ = modifier_->addOutput<ConfusionMatrixMessage>("Evaluation Result");

    confusion_ = ConfusionMatrix();
}

void MLEvaluator::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> truth_msg = in_truth_->getMessage<GenericVectorMessage, FeaturesMessage>();
    std::shared_ptr<std::vector<FeaturesMessage> const> classified_msg = in_classified_->getMessage<GenericVectorMessage, FeaturesMessage>();

    std::size_t n = truth_msg->size();

    for(std::size_t i = 0; i < n; ++i) {
        const FeaturesMessage& truth = truth_msg->at(i);
        const FeaturesMessage& classified = classified_msg->at(i);

        apex_assert_hard(truth.value.size() == classified.value.size());

        const float& t = truth.classification;
        const float& c = classified.classification;

        confusion_.reportClassification(t, c);
    }

    ConfusionMatrixMessage::Ptr result(new ConfusionMatrixMessage);
    result->confusion = confusion_;
    out_->publish(result);
}
