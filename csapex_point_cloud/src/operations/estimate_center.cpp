#include "estimate_center.hpp"

#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>
#include <csapex_point_cloud/msg/indeces_message.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/yaml_io.hpp>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>


CSAPEX_REGISTER_CLASS(csapex::EstimateCenter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

EstimateCenter::EstimateCenter()
{
    connection_types::MessageConversionHook<connection_types::GenericPointerMessage, geometry_msgs::Pose>::registerConversion();
    connection_types::MessageConversionHook<connection_types::GenericPointerMessage, geometry_msgs::PoseStamped>::registerConversion();
    connection_types::MessageConversionHook<connection_types::GenericPointerMessage, geometry_msgs::PoseWithCovariance>::registerConversion();
    connection_types::MessageConversionHook<connection_types::GenericPointerMessage, geometry_msgs::PoseWithCovarianceStamped>::registerConversion();
}

void EstimateCenter::setupParameters(Parameterizable& parameters)
{
    param::ParameterPtr bbox_param = param::ParameterFactory::declareBool("use bounding box",
                                                                          param::ParameterDescription("Use bounding box to calculate center point."),
                                                                          false);
    parameters.addParameter(bbox_param,
                            param_use_bounding_box_);
    parameters.addConditionalParameter(param::ParameterFactory::declareBool("only half visible",
                                                                            param::ParameterDescription("Assumes only half of the object is visible. Moves center point to the back plane of the bounding box"),
                                                                            false),
                                       [=]() { return bbox_param->as<bool>(); },
                                       param_use_bounding_box_);
}

void EstimateCenter::setup(NodeModifier& node_modifier)
{
    input_clouds_ = node_modifier.addInput<GenericVectorMessage, PointCloudMessage::Ptr>("PointClouds");

    output_poses_ = node_modifier.addOutput<geometry_msgs::PoseArray>("PoseArray");
    output_poses_covariance_ = node_modifier.addOutput<GenericVectorMessage, std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped>>("PoseWithCovariance");
}

void EstimateCenter::process()
{
    std::shared_ptr<std::vector<PointCloudMessage::ConstPtr> const> pcl_vector =
            msg::getMessage<GenericVectorMessage, PointCloudMessage::ConstPtr>(input_clouds_);

    output_.clear();

    for (const PointCloudMessage::ConstPtr& pcl_msg : *pcl_vector)
    {
        tmp.frame_id = pcl_msg->frame_id;
        tmp.stamp_micro_seconds = pcl_msg->stamp_micro_seconds;

        boost::apply_visitor(PointCloudMessage::Dispatch<EstimateCenter>(this, pcl_msg), pcl_msg->value);
    }

    if (output_poses_->isConnected())
    {
        geometry_msgs::PoseArray::Ptr display_pose(new geometry_msgs::PoseArray());
        display_pose->header.frame_id = tmp.frame_id;
        ros::Time now;
        now.fromNSec(tmp.stamp_micro_seconds * 1e3);
        display_pose->header.stamp = now;

        for (auto&& with_cov : output_)
            display_pose->poses.push_back(with_cov->pose.pose);

        msg::publish(output_poses_, display_pose);
    }

    if (output_poses_covariance_->isConnected())
    {
        std::shared_ptr< std::vector<std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>>>
                msgs(new std::vector<std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>> );

        *msgs = output_;

        msg::publish<GenericVectorMessage, std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>>(output_poses_covariance_, msgs);
    }
}

std::string print(const Eigen::Vector4f v)
{
    std::ostringstream os;
    os << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
    return os.str();
}

template<typename PointT>
void EstimateCenter::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    Eigen::Vector4f center;
    Eigen::Matrix3f covariance;
    if (param_use_bounding_box_)
    {
        Eigen::Vector4f min_point;
        Eigen::Vector4f max_point;

        pcl::getMinMax3D(*cloud, min_point, max_point);

        center = (min_point + max_point) / 2;

        if (param_assume_half_visible_)
        {
            // move point to the back of the bounding box
            center[0] = std::max(min_point[0], max_point[0]);
        }

        pcl::computeCovarianceMatrixNormalized(*cloud, center, covariance);
    }
    else
    {
        pcl::computeMeanAndCovarianceMatrix(*cloud, covariance, center);
    }

    ros::Time now;
    now.fromNSec(tmp.stamp_micro_seconds * 1e3);

    std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> out(new geometry_msgs::PoseWithCovarianceStamped());
    out->header.frame_id = tmp.frame_id;
    out->header.stamp = now;
    out->pose.pose.position.x = center[0];
    out->pose.pose.position.y = center[1];
    out->pose.pose.position.z = center[2];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out->pose.covariance[i * 6 + j] = covariance(i, j);

    output_.push_back(out);
}
