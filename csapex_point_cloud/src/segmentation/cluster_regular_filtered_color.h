#ifndef CLUSTERREGULARFILTEREDLCH_H
#define CLUSTERREGULARFILTEREDLCH_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

#include "regular_structures/cluster_params.hpp"
#include "regular_structures/indexation.hpp"

namespace csapex {
template<typename StructureType>
class ClusterRegularFilteredColor : public csapex::Node
{
public:
    using StructureIndex  = typename StructureType::Index;
    using IndexationType  = Indexation<StructureType>;

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input*        in_cloud_;
    Input*        in_indices_;
    Input*        in_color_;
    Output*       out_;
    Output*       out_rejected_;

    ClusterParamsStatisticalIC cluster_params_;

};
}

#endif // CLUSTERREGULARFILTEREDLCH_H
