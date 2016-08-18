#ifndef TO_FEATURE_H
#define TO_FEATURE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/generic_vector_message.hpp>

namespace csapex {

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
