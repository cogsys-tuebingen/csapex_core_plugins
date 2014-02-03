#ifndef SCALE_H
#define SCALE_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/core/core.hpp>

namespace csapex {
class Scale : public csapex::Node
{
public:
    Scale();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorOut*                    output_;
    ConnectorIn*                     input_;

    cv::Vec2d                        scales_;
    int                              mode_;
    void update();

};

}
#endif // SCALE_H
