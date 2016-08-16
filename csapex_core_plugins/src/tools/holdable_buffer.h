#ifndef HOLDABLE_BUFFER_H
#define HOLDABLE_BUFFER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/token_data.h>

/// SYSTEM
#include <deque>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN HoldableBuffer : public Node
{
public:
    HoldableBuffer();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Input  *in_;
    Output *out_;

    std::deque<TokenData::ConstPtr> buffer_;
};
}

#endif // BUFFER_H
