#ifndef SUM_CHANNELS_H
#define SUM_CHANNELS_H

/// COMPONENT
#include <csapex/model/node.h>
namespace csapex
{
class SumChannels : public csapex::Node
{
public:
    SumChannels();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;
};
}  // namespace csapex
#endif  // SUM_CHANNELS_H
