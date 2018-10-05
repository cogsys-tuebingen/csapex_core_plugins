#ifndef GROW_ROI_H
#define GROW_ROI_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class GrowROIs : public csapex::Node
{
public:
    GrowROIs();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

private:
    Input* input_;
    Output* output_;
    int x_, y_;
};

}  // namespace csapex

#endif  // GROW_ROI_H
