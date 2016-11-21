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
    virtual void setup(NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& params) override;

private:
    Output* output_;

    Input* input_a_;
    Input* input_b_;

    int which_stamp_;
};

}

#endif // TRANSFORM_COMBINER_H
