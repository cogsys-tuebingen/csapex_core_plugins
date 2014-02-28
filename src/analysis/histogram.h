#ifndef HISTOGRAM_H
#define HISTOGRAM_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class Histogram : public csapex::Node
{
public:
    Histogram();

    virtual void process();
    virtual void setup();

protected:
    ConnectorOut*   output_;
    ConnectorIn*    input_;
};
}

#endif // HISTOGRAM_H
