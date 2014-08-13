#ifndef NUMBER_GENERATOR_H
#define NUMBER_GENERATOR_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class NumberGenerator : public csapex::Node
{
public:
    NumberGenerator();

    virtual void process();
    virtual void setup();

private:
    Input* input_;
    Output* output_;

    int n;
};
}

#endif // NUMBER_GENERATOR_H
