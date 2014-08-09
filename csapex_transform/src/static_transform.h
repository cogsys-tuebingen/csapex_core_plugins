#ifndef STATIC_TRANSFORM_H
#define STATIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class StaticTransform : public csapex::Node
{
public:
    StaticTransform();

    void process();
    virtual void setup();

    virtual void tick();

private:
    Output* output_;
};

}

#endif // STATIC_TRANSFORM_H
