#ifndef SEGMENT_LABELER_H
#define SEGMENT_LABELER_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex
{
class SegmentLabeler : public csapex::Node
{
public:
    SegmentLabeler();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* in_segments_;
    Input* in_labeled_scan_;
    Output* out_;
};

}  // namespace csapex

#endif  // SEGMENT_LABELER_H
