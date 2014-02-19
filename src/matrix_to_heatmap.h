#ifndef EXTREMUM_RENDERER_H
#define EXTREMUM_RENDERER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class MatrixToHeatmap : public csapex::Node
{
public:
    MatrixToHeatmap();

    virtual void process();
    virtual void setup();

private:
    enum ColorType {BEZIER, PARABOLA};

    void update();

    ColorType    color_type_;
    ConnectorOut* output_;
    ConnectorIn* input_;
};
}
#endif // EXTREMUM_RENDERER_H
