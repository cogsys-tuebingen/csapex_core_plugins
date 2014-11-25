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

    void trigger();
    void tick();
    bool canTick();

private:
    Slot* in_;
    Output* out_;

    bool can_tick_;
};


}

#endif // OPTIMIZATION_DUMMY_H
