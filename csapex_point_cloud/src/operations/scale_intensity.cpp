/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex/msg/io.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

using namespace connection_types;


class ScaleIntensity : public Node
{
public:
    ScaleIntensity()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
        output_cloud_ = node_modifier.addOutput<PointCloudMessage>("scaled PointCloud");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareValue("scale", 1.0), scale_);
    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
        PointCloudMessage::ConstPtr out = msg;

        auto intens_cloud = boost::get<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>(msg->value);
        if(intens_cloud && scale_ != 1.0) {
            PointCloudMessage::Ptr res(new PointCloudMessage(msg->frame_id, msg->stamp_micro_seconds));
            res->value = scaleCloud(intens_cloud);
            out = res;
        }

        msg::publish(output_cloud_, out);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scaleCloud(const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in)
    {
        typename pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);

        out->header = in->header;
        out->height = in->height;
        out->width = in->width;
        out->is_dense = in->is_dense;
        out->sensor_origin_ = in->sensor_origin_;
        out->sensor_orientation_ = in->sensor_orientation_;

        out->points = in->points;

        for(auto& pt : out->points) {
            pt.intensity *= scale_;
        }

        return out;
    }

private:
    Input*  input_cloud_;
    Output* output_cloud_;
    Output* output_cloud_complement_;

    param::RangeParameter::Ptr col_;

    double scale_;
};
CSAPEX_REGISTER_CLASS(csapex::ScaleIntensity, csapex::Node)
}
