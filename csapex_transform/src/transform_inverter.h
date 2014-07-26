#ifndef TRANSFORM_INVERTER_H
#define TRANSFORM_INVERTER_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TransformInverter : public csapex::Node
{
public:
    TransformInverter();

    virtual void process();
    virtual void setup();

private:
    Input* input_;
    Output* output_;
};

}

#endif // TRANSFORM_INVERTER_H
