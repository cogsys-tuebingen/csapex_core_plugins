#ifndef TO_FEATURE_H
#define TO_FEATURE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/generic_vector_message.hpp>

namespace csapex
{
class ToFeature : public csapex::Node
{
public:
    ToFeature();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

protected:
    csapex::Input* input_;
    csapex::Output* output_;
};

}  // namespace csapex
#endif  // TO_FEATURE_H
