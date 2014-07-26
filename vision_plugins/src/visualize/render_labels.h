#ifndef RENDER_LABELS_H
#define RENDER_LABELS_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class RenderLabels : public csapex::Node
{
public:
    RenderLabels();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    csapex::Input*    labels_;
    csapex::Input*    image_;
    csapex::Output*   output_;

};
}
#endif // RENDER_LABELS_H
