#ifndef RENDER_ROIS_H
#define RENDER_ROIS_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class ExtractROI : public csapex::Node
{
public:
    ExtractROI();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    Input* input_img_;
    Input* input_roi_;

    Output* output_;
};

}
#endif // RENDER_ROIS_H
