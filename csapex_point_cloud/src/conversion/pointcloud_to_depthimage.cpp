
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

namespace csapex
{

class PointCloudToDepthImage : public Node
{
public:
    PointCloudToDepthImage()
    {
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(csapex::param::ParameterFactory::declareRange("scale", 1.0, 1000.0, 1.0, 0.5));
        parameters.addParameter(csapex::param::ParameterFactory::declareBool("fit", false));
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");

        output_ = node_modifier.addOutput<CvMatMessage>("DepthImage");
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

        boost::apply_visitor (PointCloudMessage::Dispatch<PointCloudToDepthImage>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        unsigned n = cloud->points.size();

        apex_assert(cloud->isOrganized());
        apex_assert(cloud->width != 0 && cloud->height != 0);

        int cols = cloud->width;
        int rows = n / cols;

        CvMatMessage::Ptr output(new CvMatMessage(enc::depth, cloud->header.stamp));
        output->value.create(rows,cols, CV_32F);

        typename pcl::PointCloud<PointT>::const_iterator pt = cloud->points.begin();
        float* data = (float*) output->value.data;

        double max_dist = std::numeric_limits<double>::min();
        double min_dist = std::numeric_limits<double>::max();
        for(unsigned idx = 0; idx < n; ++idx) {
            const PointT& p = *pt;
            double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);

            if(dist != dist)
                dist = 0;

            *data = dist;

            if(dist < min_dist) {
                min_dist = dist;
            }
            if(dist > max_dist) {
                max_dist = dist;
            }

            ++data;
            ++pt;
        }

        bool fit = readParameter<bool>("fit");

        double s = readParameter<double>("scale");
        if(fit) {
            s = 255.0 / (max_dist - min_dist);
        }

        output->value = (fit ? (output->value - min_dist) : output->value) * s;

        msg::publish(output_, output);
    }
private:
    Input* input_;
    Output* output_;
};

}

CSAPEX_REGISTER_CLASS(csapex::PointCloudToDepthImage, csapex::Node)
