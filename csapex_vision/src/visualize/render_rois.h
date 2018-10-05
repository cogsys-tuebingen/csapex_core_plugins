#ifndef RENDER_ROIS_H
#define RENDER_ROIS_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class RenderROIs : public csapex::Node
{
public:
    RenderROIs();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input* input_img_;
    Input* input_rois_;

    Output* output_;
};

}  // namespace csapex
#endif  // RENDER_ROIS_H
