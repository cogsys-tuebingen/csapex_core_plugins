/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
using namespace connection_types;

class ProjectZ : public Node
{
public:
    ProjectZ()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
        output_cloud_ = node_modifier.addOutput<PointCloudMessage>("scaled PointCloud");
    }

    virtual void setupParameters(Parameterizable& parameters) override
    {
    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg = msg::getMessage<PointCloudMessage>(input_cloud_);

        boost::apply_visitor(PointCloudMessage::Dispatch<ProjectZ>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        PointCloudMessage::Ptr res(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        res->value = projectCloud<PointT>(cloud);

        msg::publish(output_cloud_, res);
    }

    template <class PointT>
    typename pcl::PointCloud<PointT>::Ptr projectCloud(const typename pcl::PointCloud<PointT>::ConstPtr& in)
    {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

        out->header = in->header;
        out->height = in->height;
        out->width = in->width;
        out->is_dense = in->is_dense;
        out->sensor_origin_ = in->sensor_origin_;
        out->sensor_orientation_ = in->sensor_orientation_;

        out->points = in->points;

        for (auto& pt : out->points) {
            pt.z = 0.0;
        }

        return out;
    }

private:
    Input* input_cloud_;
    Output* output_cloud_;
    Output* output_cloud_complement_;

    param::RangeParameter::Ptr col_;

    double scale_;
};
CSAPEX_REGISTER_CLASS(csapex::ProjectZ, csapex::Node)
}  // namespace csapex
