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

    csapex::Output*   out_pyr_;
    csapex::Output*   out_level_;
    csapex::Input*    input_;

    void update();
};
}

#endif // IMAGE_PYRAMID_H
