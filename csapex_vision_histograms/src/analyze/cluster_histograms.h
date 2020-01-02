#ifndef CLUSTERHISTOGRAMS_H
#define CLUSTERHISTOGRAMS_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class ClusterHistograms : public csapex::Node
{
public:
    ClusterHistograms();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* out_histograms_;
    csapex::Output* out_clusters_;

    csapex::Input* input_;
    csapex::Input* clusters_;
    csapex::Input* mask_;
};
}  // namespace csapex
#endif  // CLUSTERHISTOGRAMS_H
