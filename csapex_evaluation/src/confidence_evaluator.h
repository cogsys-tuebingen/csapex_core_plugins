#ifndef CONFIDENCE_EVALUATOR_H
#define CONFIDENCE_EVALUATOR_H

/// COMPONENT
#include <csapex_evaluation/confidence_matrix_message.h>
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class ConfidenceEvaluator : public csapex::Node
{
public:
    ConfidenceEvaluator();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* in_classified_;
    Input* in_truth_;
    Output* out_;

    ConfidenceMatrix confidence_;
};

}  // namespace csapex

#endif  // CONFIDENCE_EVALUATOR_H
