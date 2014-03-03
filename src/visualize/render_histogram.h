#ifndef RENDER_HISTOGRAM_H
#define RENDER_HISTOGRAM_H

/// COMPONENT
#include <csapex/model/node.h>
#include <opencv2/core/core.hpp>

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

    cv::Size size_;

};
}
#endif // RENDER_HISTOGRAM_H
