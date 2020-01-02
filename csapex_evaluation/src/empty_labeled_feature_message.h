#ifndef TO_FEATURE_H
#define TO_FEATURE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/generic_vector_message.hpp>

namespace csapex
{
class EmptyLabeledFeaturesMessage : public Node
{
public:
    EmptyLabeledFeaturesMessage();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

protected:
    csapex::Output* output_;
    csapex::Output* output_vec_;
};

}  // namespace csapex
#endif  // TO_FEATURE_H
