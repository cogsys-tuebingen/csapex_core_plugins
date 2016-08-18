#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

class CornerLineDetection : public csapex::Node
{
public:
    CornerLineDetection();

    virtual void setup(csapex::NodeModifier& node_modifier) override;

protected:
    csapex::Output*                    output_;
    csapex::Input*                     input_;
};

}
#endif // CORNER_DETECTION_H
