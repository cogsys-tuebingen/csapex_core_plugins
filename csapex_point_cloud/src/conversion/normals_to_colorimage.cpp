
/// PROJECT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/normals_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

class NormalsToColorImage : public Node
{
public:
    NormalsToColorImage()
    {
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_ = node_modifier.addInput<NormalsMessage>("PointCloud");

        output_ = node_modifier.addOutput<CvMatMessage>("Image");
    }

    void process()
    {
        NormalsMessage::ConstPtr msg(msg::getMessage<NormalsMessage>(input_));

        pcl::PointCloud<pcl::Normal>::ConstPtr cloud = msg->value;

        unsigned n = cloud->points.size();

        if(n == 0)
            return;

        if(cloud->width * cloud->height != n) {
            throw std::logic_error("the input cloud is not correctly formated, width * height != count");
        }

        int cols = cloud->width;
        int rows = n / cols;

        CvMatMessage::Ptr output(new CvMatMessage(enc::bgr, cloud->header.stamp));
        output->value.create(rows,cols, CV_8UC3);

        typename pcl::PointCloud<pcl::Normal>::const_iterator pt = cloud->points.begin();
        cv::Vec3b* data = (cv::Vec3b*) output->value.data;

        for(unsigned idx = 0; idx < n; ++idx) {
            const pcl::Normal& p = *pt;
            cv::Vec3b& val = *data;
            val[0] = std::abs(p.normal_z) * 255;
            val[1] = std::abs(p.normal_y) * 255;
            val[2] = std::abs(p.normal_x) * 255;

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

CSAPEX_REGISTER_CLASS(csapex::NormalsToColorImage, csapex::Node)

