#ifndef GROW_ROI_H
#define GROW_ROI_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class GrowROI : public csapex::Node
{
public:
    GrowROI();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    Input* input_;
    Output* output_;
    int x_, y_;
};

}  // namespace csapex

#endif  // GROW_ROI_H
