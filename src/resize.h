#ifndef RESIZE_H
#define RESIZE_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/core/core.hpp>

namespace csapex {
class Resize : public csapex::Node
{
public:
    Resize();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorOut*                    output_;
    ConnectorIn*                     input_;

    cv::Size                         size_;
    int                              mode_;
    void update();
};

}
#endif // RESIZE_H
