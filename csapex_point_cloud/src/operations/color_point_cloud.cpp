/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class ColorPointCloud : public Node
{
public:
    ColorPointCloud()
    {
    }

    void setupParameters(Parameterizable& parameters) override
    {
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
        input_image_ = node_modifier.addInput<CvMatMessage>("Image");
        output_cloud_ = node_modifier.addOutput<PointCloudMessage>("Colored Cloud");
    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
        boost::apply_visitor(PointCloudMessage::Dispatch<ColorPointCloud>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        if (!cloud->isOrganized())
            throw std::runtime_error("Clouds have to be organized!");

        CvMatMessage::ConstPtr input_image = msg::getMessage<CvMatMessage>(input_image_);
        if (cloud->height != static_cast<std::size_t>(input_image->value.rows))
            throw std::runtime_error("Cloud and image have to match in height!");
        if (cloud->width != static_cast<std::size_t>(input_image->value.cols))
            throw std::runtime_error("Cloud and image have to match in width!");
        if (!input_image->getEncoding().matches(enc::bgr) && !input_image->getEncoding().matches(enc::mono))
            throw std::runtime_error("Input image must be of encoding bgr!");

        typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
        out->header = cloud->header;
        out->height = cloud->height;
        out->width = cloud->width;

        const std::size_t size = cloud->height * cloud->width;
        out->resize(size);

        cv::Mat color_data;
        if (input_image->getEncoding().matches(enc::mono)) {
            cv::cvtColor(input_image->value, color_data, cv::COLOR_GRAY2BGR);
        } else {
            color_data = input_image->value.clone();
        }

        const PointT* cloud_ptr = cloud->points.data();
        const cv::Vec3b* image_ptr = color_data.ptr<cv::Vec3b>();
        pcl::PointXYZRGB* out_ptr = out->points.data();
        for (std::size_t i = 0; i < size; ++i, ++out_ptr, ++image_ptr, ++cloud_ptr) {
            const cv::Vec3b& color = *image_ptr;
            const PointT& cloud_point = *cloud_ptr;
            pcl::PointXYZRGB& out_point = *out_ptr;

            out_point.x = cloud_point.x;
            out_point.y = cloud_point.y;
            out_point.z = cloud_point.z;
            out_point.b = color[0];
            out_point.g = color[1];
            out_point.r = color[2];
        }

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_cloud_, msg);
    }

private:
    Input* input_cloud_;
    Input* input_image_;
    Output* output_cloud_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::ColorPointCloud, csapex::Node)
