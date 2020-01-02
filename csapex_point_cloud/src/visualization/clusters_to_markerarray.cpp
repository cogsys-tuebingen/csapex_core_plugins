/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/profiling/trace.hpp>
#include <csapex/profiling/timer.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/color.hpp>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

using namespace csapex::connection_types;

namespace csapex
{
namespace impl
{
template <class PointT>
struct Impl;
}

class ClustersToMarkerArray : public Node
{
public:
    ClustersToMarkerArray() : last_count_(0), pca_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");
        in_indices_ = modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Indices");

        out_ = modifier.addOutput<visualization_msgs::MarkerArray>("Clustered Cloud");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareBool("PCA", false), pca_);
    }

    void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor(PointCloudMessage::Dispatch<ClustersToMarkerArray>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr)
    {
        const pcl::PointCloud<PointT>& cloud = *cloud_ptr;

        auto cluster_indices = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_indices_);
        const std::vector<pcl::PointIndices>& indices = *cluster_indices;

        visualization_msgs::MarkerArray::Ptr marker_array(new visualization_msgs::MarkerArray);

        visualization_msgs::Marker marker;
        marker.id = 0;
        marker.header.frame_id = cloud.header.frame_id;
        marker.header.stamp.fromNSec(cloud.header.stamp * 1e3);
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.ns = "object_aabb";
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        marker.color.r = 1.0;
        marker.color.b = marker.color.g = 0;
        marker.color.a = 0.5;

        if (pca_) {
            for (const pcl::PointIndices& cluster : indices) {
                //                new approach:
                //                    // transform everything into x-y plane
                //                    // find orientation using pca
                //                    // rotate points
                //                    // find aabb
                //                    // rotate aabb back to robot coordinates

                // transform into x-y plane
                typename pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(cloud, cluster, *xy_cloud);

                for (pcl::PointXYZ& pt : *xy_cloud) {
                    pt.z = 0.0;
                }

                // find orientation using pca
                pcl::PCA<pcl::PointXYZ> pca;
                pca.setInputCloud(xy_cloud);

                Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
                //                Eigen::Vector3f eigen_values = pca.getEigenValues();
                //                Eigen::Vector4f mean = pca.getMean();

                Eigen::Vector3f principal_component = eigen_vectors.block<3, 1>(0, 0);
                double theta = std::atan2(principal_component(1), principal_component(0));

                // rotate points
                for (pcl::PointXYZ& pt : *xy_cloud) {
                    pcl::PointXYZ rotated;
                    rotated.x = std::cos(-theta) * pt.x - std::sin(-theta) * pt.y;
                    rotated.y = std::sin(-theta) * pt.x + std::cos(-theta) * pt.y;
                    rotated.z = pt.z;
                    pt = rotated;
                }

                // find aabb
                double x_min = std::numeric_limits<double>::infinity();
                double x_max = -std::numeric_limits<double>::infinity();
                double y_min = std::numeric_limits<double>::infinity();
                double y_max = -std::numeric_limits<double>::infinity();

                for (const pcl::PointXYZ& p : *xy_cloud) {
                    if (p.x < x_min)
                        x_min = p.x;
                    if (p.x > x_max)
                        x_max = p.x;
                    if (p.y < y_min)
                        y_min = p.y;
                    if (p.y > y_max)
                        y_max = p.y;
                }

                double z_min = std::numeric_limits<double>::infinity();
                double z_max = -std::numeric_limits<double>::infinity();

                std::size_t n = cluster.indices.size();
                for (std::size_t k = 0; k < n; ++k) {
                    const PointT& p = cloud.at(cluster.indices[k]);
                    if (p.z < z_min)
                        z_min = p.z;
                    if (p.z > z_max)
                        z_max = p.z;
                }

                // rotate aabb back to robot coordinates
                double mx = (x_min + x_max) / 2.0;
                double my = (y_min + y_max) / 2.0;
                marker.pose.position.x = std::cos(theta) * mx - std::sin(theta) * my;
                marker.pose.position.y = std::sin(theta) * mx + std::cos(theta) * my;
                marker.pose.position.z = (z_min + z_max) / 2.0;

                marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

                marker.scale.x = (x_max - x_min);
                marker.scale.y = (y_max - y_min);
                marker.scale.z = (z_max - z_min);

                double r = 0, g = 0, b = 0;
                color::fromCount(marker.id, r, g, b);
                marker.color.r = r / 255.;
                marker.color.g = g / 255.;
                marker.color.b = b / 255.;

                marker_array->markers.push_back(marker);
                marker.id++;

                //                pcl::PCA<PointT> pca;
                //                pcl::PointIndicesPtr cluster_ptr(new
                //                pcl::PointIndices(cluster));
                //                pca.setIndices(cluster_ptr);
                //                pca.setInputCloud(cloud_ptr);

                //                Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
                //                Eigen::Vector3f eigen_values = pca.getEigenValues();
                //                Eigen::Vector4f mean = pca.getMean();

                //                Eigen::Vector3f x = eigen_vectors.block<3,1>(0,0);
                //                Eigen::Vector3f y = eigen_vectors.block<3,1>(0,1);
                //                Eigen::Vector3f z = x.cross(y);

                //                Eigen::Matrix3f basis = eigen_vectors;
                //                eigen_vectors.block<3,1>(0,2) = z;

                //                tf::Matrix3x3 basis_tf(basis(0,0), basis(0,1),
                //                basis(0,2),
                //                                       basis(1,0), basis(1,1),
                //                                       basis(1,2), basis(2,0),
                //                                       basis(2,1), basis(2,2));
                //                tf::Quaternion rot;
                //                basis_tf.getRotation(rot);

                //                tf::quaternionTFToMsg(rot, marker.pose.orientation);

                //                marker.pose.position.x = mean(0);
                //                marker.pose.position.y = mean(1);
                //                marker.pose.position.z = mean(2);

                //                marker.scale.x = eigen_values(0);
                //                marker.scale.y = eigen_values(1);
                //                marker.scale.z = eigen_values(2);
                //                marker.scale.x = 1;
                //                marker.scale.y = 0.25;
                //                marker.scale.z = 0.5;

                //                marker_array->markers.push_back(marker);
                //                marker.id ++;
            }

        } else {
            for (const pcl::PointIndices& cluster : indices) {
                double x_min = std::numeric_limits<double>::infinity();
                double x_max = -std::numeric_limits<double>::infinity();
                double y_min = std::numeric_limits<double>::infinity();
                double y_max = -std::numeric_limits<double>::infinity();
                double z_min = std::numeric_limits<double>::infinity();
                double z_max = -std::numeric_limits<double>::infinity();

                std::size_t n = cluster.indices.size();
                for (std::size_t k = 0; k < n; ++k) {
                    const PointT& p = cloud.at(cluster.indices[k]);

                    if (p.x < x_min)
                        x_min = p.x;
                    if (p.x > x_max)
                        x_max = p.x;
                    if (p.y < y_min)
                        y_min = p.y;
                    if (p.y > y_max)
                        y_max = p.y;
                    if (p.z < z_min)
                        z_min = p.z;
                    if (p.z > z_max)
                        z_max = p.z;
                }

                marker.pose.position.x = (x_min + x_max) / 2.0;
                marker.pose.position.y = (y_min + y_max) / 2.0;
                marker.pose.position.z = (z_min + z_max) / 2.0;

                marker.scale.x = (x_max - x_min);
                marker.scale.y = (y_max - y_min);
                marker.scale.z = (z_max - z_min);

                marker_array->markers.push_back(marker);
                marker.id++;
            }
        }

        marker.action = visualization_msgs::Marker::DELETE;
        for (int i = marker.id, n = last_count_; i < n; ++i) {
            marker.id = i;
            marker_array->markers.push_back(marker);
        }

        last_count_ = marker.id;

        msg::publish<visualization_msgs::MarkerArray>(out_, marker_array);
    }

private:
    Input* in_;
    Input* in_indices_;

    Output* out_;

    int last_count_;
    bool pca_;
};

namespace impl
{
template <class PointT>
struct Impl
{
    static void inputCloud(ClustersToMarkerArray* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        instance->inputCloud(cloud);
    }
};

}  // namespace impl

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::ClustersToMarkerArray, csapex::Node)
