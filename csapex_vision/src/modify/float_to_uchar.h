#ifndef FLOAT_TO_UCHAR_H
#define FLOAT_TO_UCHAR_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class FloatToUchar : public csapex::Node
{
public:
    FloatToUchar();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Input* input_;
    csapex::Output* output_;
};
}  // namespace csapex
#endif  // FLOAT_TO_UCHAR_H
