#ifndef SPLIT_LABELED_SCAN_H
#define SPLIT_LABELED_SCAN_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex
{
class SplitLabeledScan : public csapex::Node
{
public:
    SplitLabeledScan();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    Input* in_;
    Output* out_scan_;
    Output* out_labels_;
};

}  // namespace csapex

#endif  // SPLIT_LABELED_SCAN_H
