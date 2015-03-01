#ifndef TRANSFORM_COMBINER_H
#define TRANSFORM_COMBINER_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TransformCombiner : public csapex::Node
{
public:
    TransformCombiner();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

private:
    Output* output_;

    Input* input_a_;
    Input* input_b_;
};

}

#endif // TRANSFORM_COMBINER_H
