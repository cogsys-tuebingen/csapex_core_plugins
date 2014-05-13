#ifndef EQUALIZE_H
#define EQUALIZE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class Equalize : public csapex::Node
{
public:
    Equalize();

    virtual void process();
    virtual void setup();

protected:
    csapex::ConnectorOut*   output_;
    csapex::ConnectorIn*    input_;
};

}

#endif // EQUALIZE_H
