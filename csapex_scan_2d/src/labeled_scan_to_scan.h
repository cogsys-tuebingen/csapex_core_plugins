#ifndef LABELED_SCAN_TO_SCAN_H
#define LABELED_SCAN_TO_SCAN_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class LabeledScanToScan : public Node
{
public:
    LabeledScanToScan();

    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* in_;
    Output* out_;
};
}

#endif // LABELED_SCAN_TO_SCAN_H
