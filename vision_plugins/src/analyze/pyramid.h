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
    int  amount_levels_;
    int  preview_level_;

    csapex::ConnectorOut*   levels_;
    csapex::ConnectorOut*   preview_;
    csapex::ConnectorIn*    input_;

    void update();
};
}

#endif // IMAGE_PYRAMID_H
