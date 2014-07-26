/// HEADER
#include "point_count.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::PointCount, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PointCount::PointCount()
{
}

void PointCount::setup()
{
    input_ = modifier_->addInput<PointCloudMessage>("PointCloud");
}

void PointCount::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<PointCount>(this, msg), msg->value);
}

template <class PointT>
void PointCount::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    int c = cloud->points.size();
    display_request(c);
}
