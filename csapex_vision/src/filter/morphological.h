#ifndef MORPOLOCIAL_H
#define MORPOLOCIAL_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class Morpholocial : public csapex::Node
{
public:
    Morpholocial();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Output* output_;

    Input* input_;
};
}

#endif // MORPOLOCIAL_H
