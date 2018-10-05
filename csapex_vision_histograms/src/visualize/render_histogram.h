#ifndef RENDER_HISTOGRAM_H
#define RENDER_HISTOGRAM_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class RenderHistogram : public csapex::Node
{
public:
    RenderHistogram();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Output* output_;
    csapex::Input* input_;
    csapex::Input* maxima_;

    void update();

    int height_;
    int width_;
};
}  // namespace csapex
#endif  // RENDER_HISTOGRAM_H
