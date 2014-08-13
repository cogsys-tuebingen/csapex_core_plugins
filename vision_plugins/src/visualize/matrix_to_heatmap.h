#ifndef EXTREMUM_RENDERER_H
#define EXTREMUM_RENDERER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class MatrixToHeatmap : public csapex::Node
{
public:
    MatrixToHeatmap();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:
    enum ColorType {BEZIER, PARABOLA};

    void update();

    ColorType             color_type_;
    csapex::Output* output_;
    csapex::Input*  input_;
    csapex::Input*  mask_;
};
}
#endif // EXTREMUM_RENDERER_H
