/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/msg/indeces_message.h>
#include <csapex/model/node.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_opencv/cv_mat_message.h>

namespace csapex {
using namespace connection_types;

class PointIndicesToMask : public Node
{
public:
    void setup(NodeModifier &node_modifier) override
    {
        input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
        input_indices_ = node_modifier.addInput<PointIndecesMessage>("PointIndices");

        output_mask_ = node_modifier.addOutput<CvMatMessage>("Mask");
    }
    void setupParameters(Parameterizable &parameters) override
    {
    }
    void process() override
    {
        PointCloudMessage::ConstPtr pointcloud(msg::getMessage<PointCloudMessage>(input_cloud_));
        boost::apply_visitor(PointCloudMessage::Dispatch<PointIndicesToMask>(this, pointcloud), pointcloud->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        PointIndecesMessage::ConstPtr indices(msg::getMessage<PointIndecesMessage>(input_indices_));
        CvMatMessage::Ptr mask(new CvMatMessage(enc::mono, cloud->header.stamp));
        mask->value = cv::Mat(cloud->height, cloud->width, CV_8UC1, cv::Scalar());
        for(int index : indices->value->indices) {
            mask->value.at<uchar>(index) = 1;
        }

        msg::publish(output_mask_, mask);
    }


protected:
    Input  *input_cloud_;
    Input  *input_indices_;

    Output *output_mask_;
};

}

CSAPEX_REGISTER_CLASS(csapex::PointIndicesToMask, csapex::Node)
