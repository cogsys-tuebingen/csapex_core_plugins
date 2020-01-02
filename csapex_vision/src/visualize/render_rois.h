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

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    Input* input_img_;
    Input* input_rois_;

    Output* output_;
};

}  // namespace csapex
#endif  // RENDER_ROIS_H
