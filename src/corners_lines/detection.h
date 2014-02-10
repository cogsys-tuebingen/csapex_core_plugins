#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

class CornerDetection : public csapex::Node
{
public:
    CornerDetection();

    virtual void setup();

protected:
    ConnectorOut*                    output_;
    ConnectorIn*                     input_;
};

}
#endif // CORNER_DETECTION_H
