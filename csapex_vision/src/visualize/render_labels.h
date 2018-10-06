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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Input* labels_;
    csapex::Input* image_;
    csapex::Output* output_;
};
}  // namespace csapex
#endif  // RENDER_LABELS_H
