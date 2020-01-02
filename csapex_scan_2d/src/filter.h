#ifndef SCAN_FILTER_H
#define SCAN_FILTER_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class ScanFilter : public csapex::Node
{
public:
    ScanFilter();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

#endif  // SCAN_FILTER_H
