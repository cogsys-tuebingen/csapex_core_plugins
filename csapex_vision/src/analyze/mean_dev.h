#ifndef MEAN_H
#define MEAN_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class MeanStdDev : public Node
{
public:
    MeanStdDev();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    Input* in_mat_;
    Input* in_mask_;
    Output* out_mean_;
    Output* out_stddev_;
};
}  // namespace csapex
#endif  // MEAN_H
