#ifndef HISTOGRAM_VISION_H
#define HISTOGRAM_VISION_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class Histogram : public csapex::Node
{
public:
    Histogram();

    virtual void process();
    virtual void setup();

protected:
    csapex::ConnectorOut*   output_;
    csapex::ConnectorIn*    input_;
    csapex::ConnectorIn*    mask_;

    int  bins_;
    bool uniform_;
    bool accumulate_;

    void update();

};
}

#endif // HISTOGRAM_VISION_H
