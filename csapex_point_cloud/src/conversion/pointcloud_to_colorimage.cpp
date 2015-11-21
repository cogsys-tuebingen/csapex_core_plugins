
/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

class PointCloudToColorImage : public Node
{
public:
    PointCloudToColorImage()
    {
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");

        output_ = node_modifier.addOutput<CvMatMessage>("Image");
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

        typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud =
                boost::get<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(msg->value);

        if(cloud) {
            inputCloud(cloud);
        }
    }

    void inputCloud(typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
    {
        unsigned n = cloud->points.size();

        if(cloud->width * cloud->height != n) {
            throw std::logic_error("the input cloud is not correctly formated, width * height != count");
        }

        int cols = cloud->width;
        int rows = n / cols;

        CvMatMessage::Ptr output(new CvMatMessage(enc::bgr, cloud->header.stamp));
        output->value.create(rows,cols, CV_8UC3);

        typename pcl::PointCloud<pcl::PointXYZRGB>::const_iterator pt = cloud->points.begin();
        cv::Vec3b* data = (cv::Vec3b*) output->value.data;

        for(unsigned idx = 0; idx < n; ++idx) {
            const pcl::PointXYZRGB& p = *pt;
            cv::Vec3b& val = *data;
            val[0] = p.b;
            val[1] = p.g;
            val[2] = p.r;

            ++data;
            ++pt;
        }

        msg::publish(output_, output);
    }
private:
    Input* input_;
    Output* output_;
};

}

CSAPEX_REGISTER_CLASS(csapex::PointCloudToColorImage, csapex::Node)

