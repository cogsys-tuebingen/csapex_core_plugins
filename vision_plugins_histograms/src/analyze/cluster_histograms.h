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
    csapex::Output*   out_cluster_ids_;
    csapex::Output*   out_histograms_;

    csapex::Input*    input_;
    csapex::Input*    clusters_;
    csapex::Input*    mask_;

    int bins_;


    void update();

};
}
#endif // CLUSTERHISTOGRAMS_H
