#ifndef EXTREMUM_RENDERER_H
#define EXTREMUM_RENDERER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class ExtremumRenderer : public csapex::Node
{
public:
    ExtremumRenderer();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    void update();

    ConnectorOut* output_;
    ConnectorIn* input_;
};
}
#endif // EXTREMUM_RENDERER_H
