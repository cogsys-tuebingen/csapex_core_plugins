#ifndef IMAGE_PYRAMID_H
#define IMAGE_PYRAMID_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class Pyramid : public csapex::Node
{
public:
    Pyramid();

    virtual void process();
    virtual void setup();

protected:
    int             amount_levels_;
    int             preview_level_;

    ConnectorOut*   levels_;
    ConnectorOut*   preview_;
    ConnectorIn*    input_;

    void update();
};
}

#endif // IMAGE_PYRAMID_H
