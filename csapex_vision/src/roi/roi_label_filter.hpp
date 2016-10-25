#pragma once

#include <csapex/model/node.h>
#include <set>

namespace csapex
{
class ROILabelFilter : public Node
{
public:
    void setupParameters(csapex::Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    void updateLabels();

private:
    csapex::Input* in_rois_;
    csapex::Output* out_rois_;

    std::set<int> labels_;
};
}
