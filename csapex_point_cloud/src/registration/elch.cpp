/// HEADER
#include "elch.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/interlude.hpp>
#include <csapex/utility/timer.h>
#include <csapex_point_cloud/point_cloud_message.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/elch.h>

CSAPEX_REGISTER_CLASS(csapex::ELCH, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

/// https://github.com/PointCloudLibrary/pcl/blob/master/tools/elch.cpp

ELCH::ELCH()
{

}

void ELCH::setup(NodeModifier &node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    out_cloud_ = node_modifier.addOutput<PointCloudMessage>("Fused PointCloud");
}

void ELCH::setupParameters(Parameterizable &parameters)
{

}

void ELCH::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<ELCH>(this, msg), msg->value);
}

namespace impl {
template<typename PointT>
struct PCLELCH : public ELCH::PCLELCH {
    pcl::registration::ELCH<PointT>                 elch;
    typename pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp;

    PCLELCH(const int max_iter,
            const double correspondence_distance,
            const double ransac_outlier_thresh) :
        icp(new pcl::IterativeClosestPoint<PointT, PointT>)
    {
        icp->setMaximumIterations(max_iter);
        icp->setMaxCorrespondenceDistance(correspondence_distance);
        icp->setRANSACOutlierRejectionThreshold(ransac_outlier_thresh);
        elch.setReg(icp);
    }
};
}





template<class PointT>
void ELCH::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{

}
