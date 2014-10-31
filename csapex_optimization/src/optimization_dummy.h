#ifndef OPTIMIZATION_DUMMY_H
#define OPTIMIZATION_DUMMY_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class OptimizationDummy : public csapex::Node
{
public:
    OptimizationDummy();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_;
    Output* out_;
};


}

#endif // OPTIMIZATION_DUMMY_H
