/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/utility/timer.h>
#include <csapex/utility/interlude.hpp>

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

using namespace csapex::connection_types;


namespace csapex
{


namespace impl {

template <class PointT>
struct Impl;

}

template <typename PointT>
double distance(const PointT& a)
{
    auto dx = a.x;
    auto dy = a.y;
    auto dz = a.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
template <typename PointT>
double distanceXYZ(const PointT& a, const PointT& b)
{
    auto dx = a.x - b.x;
    auto dy = a.y - b.y;
    auto dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
template <typename PointT>
double distanceXY(const PointT& a, const PointT& b)
{
    auto dx = a.x - b.x;
    auto dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

template <typename PointT>
double distanceZ(const PointT& a, const PointT& b)
{
    auto dz = a.z - b.z;
    return std::abs(dz);
}

void merge(pcl::PointIndices& a, pcl::PointIndices& b)
{
    a.indices.insert(a.indices.end(), b.indices.begin(), b.indices.end());
    b.indices.clear();
}

template <typename PointT>
PointT calculateMean(const pcl::PointCloud<PointT>& cloud, const pcl::PointIndices& cluster)
{
    PointT m;
    m.x = 0;
    m.y = 0;
    m.z = 0;

    std::size_t n = cluster.indices.size();
    for(std::size_t k = 0; k < n; ++k) {
        const PointT& p = cloud.at(cluster.indices[k]);
        m.x += p.x;
        m.y += p.y;
        m.z += p.z;
    }

    m.x /= n;
    m.y /= n;
    m.z /= n;

    return m;
}

class MergeClusters : public Node
{
public:
    MergeClusters()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");
        in_indices_ = modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Indices");

        out_ = modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Clustered Cloud");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("distance xy", 0.0, 1.0, 0.3, 0.01),
                            cluster_distance_xy_);
        params.addParameter(param::ParameterFactory::declareRange("distance xy factor", 0.0, 10.0, 0.0, 0.01),
                            cluster_distance_xy_distance_factor_);
        params.addParameter(param::ParameterFactory::declareRange("distance xy offset", -3.0, 3.0, 0.0, 0.01),
                            cluster_distance_xy_distance_offset_);
        params.addParameter(param::ParameterFactory::declareRange("distance z", 0.0, 2.0, 0.3, 0.01),
                            cluster_distance_z_);
        params.addParameter(param::ParameterFactory::declareRange("max mean distance", 0.0, 3.0, 1.0, 0.01),
                            cluster_max_mean_xyz_);

        params.addParameter(param::ParameterFactory::declareRange("min cluster size", 0, 1000, 100, 1),
                            cluster_min_size_);
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor (PointCloudMessage::Dispatch<MergeClusters>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr)
    {
        const pcl::PointCloud<PointT>& cloud = *cloud_ptr;

        auto cluster_indices = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_indices_);
        const std::vector<pcl::PointIndices>& indices = *cluster_indices;

        auto out_indices_msg = std::make_shared<std::vector<pcl::PointIndices>>();
        std::vector<pcl::PointIndices>& merged_indices = *out_indices_msg;

        merged_indices = indices;

        std::map<std::vector<pcl::PointIndices>::iterator, PointT> mean;
        for(auto it = merged_indices.begin(); it != merged_indices.end();++it) {
            pcl::PointIndices& c1 = *it;
            mean[it] = calculateMean(cloud, c1);
        }

        // cluster euclidean
        for(auto it = merged_indices.begin(); it != merged_indices.end();++it) {
            pcl::PointIndices& c1 = *it;
            const PointT& mean1 = mean[it];
            if(!c1.indices.empty()) {
                for(auto it2 = it + 1; it2 != merged_indices.end();++it2) {
                    pcl::PointIndices& c2 = *it2;
                    if(!c2.indices.empty()) {
                        const PointT& mean2 = mean[it2];
                        double d = distance(mean2) + cluster_distance_xy_distance_offset_;
                        double factor = cluster_distance_xy_distance_factor_ == 0 ?
                                    1.0 :
                                    cluster_distance_xy_distance_factor_ * d;

                        if(distanceXYZ(mean1, mean2) < cluster_max_mean_xyz_) {
                            for(std::size_t k = 0, n = c1.indices.size(); k < n; ++k) {
                                const PointT& p1 = cloud.at(c1.indices[k]);

                                bool merged = false;
                                for(std::size_t l = 0, n = c2.indices.size(); l < n; ++l) {
                                    const PointT& p2 = cloud.at(c2.indices[l]);
                                    if(distanceXY(p1, p2) < cluster_distance_xy_ * factor &&
                                            distanceZ(p1, p2) < cluster_distance_z_) {
                                        merge(c1, c2);
                                        mean[it] = calculateMean(cloud, c1);
                                        merged = true;
                                        break;
                                    }
                                }
                                if(merged) break;
                            }
                        }
                    }
                }
            }
        }

        for(auto it = merged_indices.begin(); it != merged_indices.end();) {
            pcl::PointIndices& c = *it;
            if(c.indices.size() < cluster_min_size_) {
                it = merged_indices.erase(it);
            } else {
                ++it;
            }
        }

        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_, out_indices_msg);
    }


private:
    Input* in_;
    Input* in_indices_;

    Output* out_;

    double cluster_distance_xy_;
    double cluster_distance_xy_distance_offset_;
    double cluster_distance_xy_distance_factor_;
    double cluster_distance_z_;
    double cluster_max_mean_xyz_;
    int    cluster_min_size_;
};


namespace impl {

template <class PointT>
struct Impl
{
    static void inputCloud(MergeClusters* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        instance->inputCloud(cloud);
    }
};

}

}

CSAPEX_REGISTER_CLASS(csapex::MergeClusters, csapex::Node)

