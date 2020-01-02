
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

/// SYSTEM
#include <pcl/common/io.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class PointCloudToXYZ : public Node
{
public:
    PointCloudToXYZ()
    {
    }

    void setupParameters(Parameterizable& parameters) override
    {
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");

        output_ = node_modifier.addOutput<PointCloudMessage>("XYZ");
    }

    void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

        boost::apply_visitor(PointCloudMessage::Dispatch<PointCloudToXYZ>(this, msg), msg->value);
    }

    template <typename PointT>
    void inputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *result);

        PointCloudMessage::Ptr output(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        output->value = result;
        msg::publish(output_, output);
    }

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::PointCloudToXYZ, csapex::Node)
