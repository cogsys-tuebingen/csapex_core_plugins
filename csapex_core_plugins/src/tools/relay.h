#ifndef RELAY_H
#define RELAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/token_data.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN Relay : public Node
{
public:
    Relay();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

#endif  // RELAY_H
