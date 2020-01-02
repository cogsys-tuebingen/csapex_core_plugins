#ifndef MEAN_DEV_NORMALIZATION_H
#define MEAN_DEV_NORMALIZATION_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class MeanStdDevNormalization : public Node
{
public:
    MeanStdDevNormalization();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Input* in_mean_;
    Input* in_dev_;
    Input* in_mat_;

    Output* out_;
};
}  // namespace csapex

#endif  // MEAN_DEV_NORMALIZATION_H
