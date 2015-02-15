/// HEADER
#include "crop_box.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/mpl/for_each.hpp>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>

CSAPEX_REGISTER_CLASS(csapex::CropBox, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

CropBox::CropBox()
{
    addParameter(param::ParameterFactory::declareInterval("dx", -10.0, 10.0, -10.0, 10.0, 0.01));
    addParameter(param::ParameterFactory::declareInterval("dy", -10.0, 10.0, -10.0, 10.0, 0.01));
    addParameter(param::ParameterFactory::declareInterval("dz", -10.0, 10.0, -10.0, 10.0, 0.01));
}

void CropBox::setup()
{
    input_cloud_ = modifier_->addInput<PointCloudMessage>("PointCloud");

    output_pos_ = modifier_->addOutput<PointCloudMessage>("cropped PointCloud (+)");
    output_neg_ = modifier_->addOutput<PointCloudMessage>("cropped PointCloud (-)");
}

void CropBox::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));

    boost::apply_visitor (PointCloudMessage::Dispatch<CropBox>(this, msg), msg->value);
}

template <class PointT>
void CropBox::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::pair<double,double> dx = readParameter<std::pair<double, double> >("dx");
    std::pair<double,double> dy = readParameter<std::pair<double, double> >("dy");
    std::pair<double,double> dz = readParameter<std::pair<double, double> >("dz");

    Eigen::Vector4f min_pt_(dx.first, dy.first, dz.first, 0);
    Eigen::Vector4f max_pt_(dx.second, dy.second, dz.second, 0);

    pcl::CropBox<PointT> crop;
    crop.setMin(min_pt_);
    crop.setMax(max_pt_);
    crop.setInputCloud(cloud);

    if(msg::isConnected(output_pos_)) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        crop.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_pos_, msg);
    }

    if(msg::isConnected(output_neg_)) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        crop.setNegative(true);
        crop.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_neg_, msg);
    }

}
