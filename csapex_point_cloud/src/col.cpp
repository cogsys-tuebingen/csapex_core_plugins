/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/msg/io.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

using namespace connection_types;


class StructuredCol : public Node
{
public:
    StructuredCol()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
        output_cloud_ = node_modifier.addOutput<PointCloudMessage>("Col");
        output_cloud_complement_ = node_modifier.addOutput<PointCloudMessage>("Complement");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {

        param::Parameter::Ptr row_ptr = param::ParameterFactory::declareRange("col", 0, 0, 0, 1);
        col_ = std::dynamic_pointer_cast<param::RangeParameter>(row_ptr);
        addParameter(col_);
    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
        boost::apply_visitor (PointCloudMessage::Dispatch<StructuredCol>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        if(!cloud->isOrganized())
            throw std::runtime_error("Clouds have to be organized!");

        if(col_->max<int>() == 0 ||
                col_->max<int>() != cloud->width) {
            col_->setMax<int>(cloud->width);
        }

        int selected = col_->as<int>();
        if(msg::isConnected(output_cloud_)) {
            typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
            out->header = cloud->header;
            out->height = cloud->height;
            out->width  = 1;

            for(std::size_t i = 0 ; i < cloud->height; ++i) {
                out->points.push_back(cloud->at(selected, i));
            }

            PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
            msg->value = out;
            msg::publish(output_cloud_, msg);
        }

        if(msg::isConnected(output_cloud_complement_)) {
            typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
            out->header = cloud->header;
            out->height = cloud->height;
            out->width  = cloud->width;

            for(std::size_t i = 0 ; i < cloud->height ; ++i) {
                for(std::size_t j = 0 ; j < cloud->width ; ++j) {
                    PointT p = cloud->at(j,i);
                    if(j != selected) {
                        out->push_back(p);
                    } else {
                        p.x = std::numeric_limits<float>::quiet_NaN();
                        p.y = std::numeric_limits<float>::quiet_NaN();
                        p.z = std::numeric_limits<float>::quiet_NaN();
                        out->push_back(p);
                    }
                }
            }


            PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
            msg->value = out;
            msg::publish(output_cloud_complement_, msg);
        }

    }

private:
    Input*  input_cloud_;
    Output* output_cloud_;
    Output* output_cloud_complement_;

    param::RangeParameter::Ptr col_;

};
CSAPEX_REGISTER_CLASS(csapex::StructuredCol, csapex::Node)
}
