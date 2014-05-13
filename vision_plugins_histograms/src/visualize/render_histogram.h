#ifndef RENDER_HISTOGRAM_H
#define RENDER_HISTOGRAM_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class RenderHistogram : public csapex::Node
{
public:
    RenderHistogram();

    virtual void process();
    virtual void setup();

protected:
    csapex::ConnectorOut*   output_;
    csapex::ConnectorIn*    input_;

    void update();

    int height_;
    int width_;

};
}
#endif // RENDER_HISTOGRAM_H
