#ifndef FilterBlur_H
#define FilterBlur_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class BoxBlur : public Node
{
public:
    BoxBlur();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:

    Input  *input_;
    Output *output_;
};

} /// NAMESPACE

#endif // FilterBlur_H
