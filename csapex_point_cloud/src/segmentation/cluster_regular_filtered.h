#ifndef CLUSTERREGULARFILTERED_H
#define CLUSTERREGULARFILTERED_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

#include "regular_structures/cluster_params.hpp"
#include "regular_structures/indexation.hpp"
#include "regular_structures/entry.hpp"

namespace csapex {
template<typename StructureType>
class ClusterRegularFiltered : public csapex::Node
{
public:
    using StructureIndex  = typename StructureType::Index;
    using IndexationType  = Indexation<StructureType>;

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);
    void validateVoxel(EntryStatistical &e);

private:
    Input*        in_cloud_;
    Input*        in_indices_;
    Output*       out_;
    Output*       out_rejected_;
    Output*       out_voxels_;

    ClusterParamsStatistical cluster_params_;

};
}

#endif // CLUSTERREGULARFILTERED_H
