#ifndef STATIC_TRANSFORM_H
#define STATIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class StaticTransform : public csapex::Node
{
public:
    StaticTransform();

    virtual void tick() override;
    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

private:
    Output* output_;
};

}

#endif // STATIC_TRANSFORM_H
