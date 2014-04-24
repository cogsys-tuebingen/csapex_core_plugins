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

protected:
    csapex::ConnectorOut *output_;
    csapex::ConnectorIn  *input_;
    csapex::ConnectorIn  *mask_;
};
}

#endif // NORMALIZE_H
