#ifndef MERGE_ROIS_H
#define MERGE_ROIS_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class MergeROIs : public csapex::Node
{
public:
    MergeROIs();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

#endif  // MERGE_ROIS_H
