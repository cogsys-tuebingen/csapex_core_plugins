#ifndef RENDER_LABELS_H
#define RENDER_LABELS_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class RenderLabels : public csapex::Node
{
public:
    RenderLabels();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Input* labels_;
    csapex::Input* image_;
    csapex::Output* output_;
};
}  // namespace csapex
#endif  // RENDER_LABELS_H
