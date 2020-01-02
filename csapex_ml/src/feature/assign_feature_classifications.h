#ifndef ASSIGNCLASS_H
#define ASSIGNCLASS_H

#include <csapex/model/node.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN AssignFeatureClassifications : public Node
{
public:
    AssignFeatureClassifications();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    csapex::Input* in_features_;
    csapex::Input* in_labels_;
    csapex::Output* out_;

    int label_;
};
}  // namespace csapex

#endif  // ASSIGNCLASS_H
