/// HEADER
#include "point_count.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::PointCount, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PointCount::PointCount()
{
}

void PointCount::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    output_ = node_modifier.addOutput<double>("count");
}

void PointCount::process()
{
    PointCloudMessage::ConstPtr msg = msg::getMessage<PointCloudMessage>(input_);

    boost::apply_visitor (PointCloudMessage::Dispatch<PointCount>(this, msg), msg->value);
}

template <class PointT>
void PointCount::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    int c = cloud->points.size();
    msg::publish(output_, c);
    display_request(c);
}
