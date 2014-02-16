#ifndef EXTREMUM_RENDERER_H
#define EXTREMUM_RENDERER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class MatrixToHeatmap : public csapex::Node
{
public:
    MatrixToHeatmap();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    void update();

    ConnectorOut* output_;
    ConnectorIn* input_;
};
}
#endif // EXTREMUM_RENDERER_H
