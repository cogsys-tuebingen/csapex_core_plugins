#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class CornerLineDetection : public csapex::Node
{
public:
    CornerLineDetection();

    void setup(csapex::NodeModifier& node_modifier) override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;
};

}  // namespace csapex
#endif  // CORNER_DETECTION_H
