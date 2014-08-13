#ifndef NORMALIZE_H
#define NORMALIZE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class Normalize : public csapex::Node
{
public:
    Normalize();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    csapex::Output *output_;
    csapex::Input  *input_;
    csapex::Input  *mask_;
};
}

#endif // NORMALIZE_H
