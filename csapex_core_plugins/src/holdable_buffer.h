#ifndef HOLDABLE_BUFFER_H
#define HOLDABLE_BUFFER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <deque>

namespace csapex {
class HoldableBuffer : public Node
{
public:
    HoldableBuffer();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Input  *in_;
    Output *out_;

    std::deque<ConnectionType::ConstPtr> buffer_;
};
}

#endif // BUFFER_H
