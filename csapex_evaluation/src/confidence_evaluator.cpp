/// HEADER
#include "confidence_evaluator.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/signal/slot.h>

CSAPEX_REGISTER_CLASS(csapex::ConfidenceEvaluator, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ConfidenceEvaluator::ConfidenceEvaluator()
{
}

void ConfidenceEvaluator::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("reset"), [&](csapex::param::Parameter*) { confidence_.reset(); });

    std::map<std::string, int> eval_types = {
        {"mean", ConfidenceMatrix::MEAN},
        {"argmax", ConfidenceMatrix::ARGMAX}
    };
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet("evaluation",
                                                                                 csapex::param::ParameterDescription("Choose the evaluation type."),
                                                                                 eval_types,
                                                                                 (int) ConfidenceMatrix::MEAN));
}

void ConfidenceEvaluator::setup(NodeModifier& node_modifier)
{
    in_truth_       = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("True feature");
    in_classified_  = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Classified feature");
    out_            = node_modifier.addOutput<ConfidenceMatrixMessage>("Evaluation Result");
    confidence_     = ConfidenceMatrix();
}

void ConfidenceEvaluator::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> truth_msg = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_truth_);
    std::shared_ptr<std::vector<FeaturesMessage> const> classified_msg = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_classified_);

    std::size_t n = truth_msg->size();

    ConfidenceMatrix::UpdateType update = (ConfidenceMatrix::UpdateType) readParameter<int>("evaluation");
    if(confidence_.update_type != update) {
        confidence_.reset(update);
    }

    for(std::size_t i = 0; i < n; ++i) {
        const FeaturesMessage& truth = truth_msg->at(i);
        const FeaturesMessage& classified = classified_msg->at(i);

        apex_assert(truth.value.size() == classified.value.size());

        const int& t = truth.classification;
        const int& c = classified.classification;

        // items with an invalid label are simply dropped so they dont influence the evaluation
        if(c == FeaturesMessage::INVALID_LABEL || t == FeaturesMessage::INVALID_LABEL) {
            continue;
        }

        confidence_.reportConfidence(t, c, classified.confidence);
    }

    ConfidenceMatrixMessage::Ptr result(new ConfidenceMatrixMessage);
    result->confidence = confidence_;
    msg::publish(out_, result);
}
