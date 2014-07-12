#ifndef IMAGE_PYRAMID_H
#define IMAGE_PYRAMID_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class Pyramid : public csapex::Node
{
public:
    Pyramid();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    int                     out_levels_;
    int                     out_level_idx_;

    csapex::ConnectorOut*   out_pyr_;
    csapex::ConnectorOut*   out_level_;
    csapex::ConnectorIn*    input_;

    void update();
};
}

#endif // IMAGE_PYRAMID_H
