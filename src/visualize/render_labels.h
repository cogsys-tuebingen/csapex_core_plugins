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

protected:
    csapex::ConnectorIn*    labels_;
    csapex::ConnectorIn*    image_;
    csapex::ConnectorOut*   output_;

};
}
#endif // RENDER_LABELS_H
