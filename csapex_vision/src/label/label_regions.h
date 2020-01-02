#ifndef LABEL_REGIONS_H
#define LABEL_REGIONS_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class LabelRegions : public csapex::Node
{
public:
    LabelRegions();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;
};
}  // namespace csapex

#endif  // LABEL_REGIONS_H
