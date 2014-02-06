#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

class CornerDetection : public csapex::Node
{
public:
    CornerDetection();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorOut*                    output_;
    ConnectorIn*                     input_;

    void update();
    double k_;
    int    k_size_;
    int    block_size_;

};

}
#endif // CORNER_DETECTION_H
