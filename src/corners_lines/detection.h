#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {

class CornerLineDetection : public csapex::Node
{
public:
    CornerLineDetection();

    virtual void setup();

protected:
    csapex::ConnectorOut*                    output_;
    csapex::ConnectorIn*                     input_;
};

}
#endif // CORNER_DETECTION_H
