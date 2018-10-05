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

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input* in_mat_;
    Input* in_mask_;
    Output* out_mean_;
    Output* out_stddev_;
};
}  // namespace csapex
#endif  // MEAN_H
