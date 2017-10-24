
/// PROJECT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace
{
template<typename PointT>
struct Converter
{
    static CvMatMessage::Ptr convert(const typename pcl::PointCloud<PointT>::ConstPtr&)
    {
        throw std::runtime_error(std::string("point type '") + type2name(typeid(PointT)) + "' not supported");
    }
};

template<>
struct Converter<pcl::PointXYZI>
{
    static CvMatMessage::Ptr convert(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
    {
        CvMatMessage::Ptr output(new CvMatMessage(enc::bgr, cloud->header.frame_id, cloud->header.stamp));
        output->value.create(cloud->height, cloud->width, CV_16UC1);

        std::transform(cloud->points.begin(), cloud->points.end(), output->value.begin<ushort>(),
                [](const pcl::PointXYZI& pt)
                {
                    return static_cast<ushort>(pt.intensity);
                });

        return output;
    }
};

template<>
struct Converter<pcl::PointXYZRGB>
{
    static CvMatMessage::Ptr convert(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
    {
        CvMatMessage::Ptr output(new CvMatMessage(enc::bgr, cloud->header.frame_id, cloud->header.stamp));
        output->value.create(cloud->height, cloud->width, CV_8UC3);

        std::transform(cloud->points.begin(), cloud->points.end(), output->value.begin<cv::Vec3b>(),
                [](const pcl::PointXYZRGB& pt)
                {
                    return cv::Vec3b(pt.b, pt.g, pt.r);
                });

        return output;
    }
};
}

namespace csapex
{

class PointCloudToImage : public Node
{
public:
    PointCloudToImage()
    {}

    void setupParameters(Parameterizable &parameters) override
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_  = node_modifier.addInput<PointCloudMessage>("PointCloud");
        output_ = node_modifier.addOutput<CvMatMessage>("Image");
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

        boost::apply_visitor(PointCloudMessage::Dispatch<PointCloudToImage>(this, msg), msg->value);
    }

    template<typename PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        auto msg = Converter<PointT>::convert(cloud);
        msg::publish(output_, msg);
    }
private:
    Input* input_;
    Output* output_;
};

}

CSAPEX_REGISTER_CLASS(csapex::PointCloudToImage, csapex::Node)

