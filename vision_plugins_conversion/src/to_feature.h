#ifndef TO_FEATURE_H
#define TO_FEATURE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>
#include <csapex_core_plugins/vector_message.h>

namespace vision_plugins {

class ToFeature : public csapex::Node
{
public:
    ToFeature();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

protected:
    csapex::Input   *input_;
    csapex::Output  *output_;
};

}
#endif // TO_FEATURE_H
