#ifndef CLUSTERHISTOGRAMS_H
#define CLUSTERHISTOGRAMS_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class ClusterHistograms : public csapex::Node
{
public:
    ClusterHistograms();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Output*   out_histograms_;
    csapex::Output*   out_clusters_;

    csapex::Input*    input_;
    csapex::Input*    clusters_;
    csapex::Input*    mask_;

};
}
#endif // CLUSTERHISTOGRAMS_H
