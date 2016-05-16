#ifndef TO_FEATURE_H
#define TO_FEATURE_H

/// PROJECT
#include <csapex/model/tickable_node.h>
#include <csapex/model/token_data.h>
#include <csapex_core_plugins/vector_message.h>

namespace csapex {

class EmptyLabeledFeaturesMessage : public TickableNode
{
public:
    EmptyLabeledFeaturesMessage();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;
    virtual void tick() override;

    virtual bool canTick() override;

protected:
    csapex::Output *output_;
    csapex::Output *output_vec_;

};

}
#endif // TO_FEATURE_H
