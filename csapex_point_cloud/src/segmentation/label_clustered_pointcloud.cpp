/// HEADER
#include "label_clustered_pointcloud.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

CSAPEX_REGISTER_CLASS(csapex::LabelClusteredPointCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

#define FLOOD_DEFAULT_LABEL 0

LabelClusteredPointCloud::LabelClusteredPointCloud()
{
    addTag(Tag::get("PointCloud"));
}

void LabelClusteredPointCloud::process()
{
    PointCloudMessage::Ptr cloud(input_->getMessage<PointCloudMessage>());

    cluster_indices = in_indices_->getMessage<GenericVectorMessage, pcl::PointIndices>();

    boost::apply_visitor (PointCloudMessage::Dispatch<LabelClusteredPointCloud>(this, cloud), cloud->value);
}

void LabelClusteredPointCloud::setup()
{
    input_  = modifier_->addInput<PointCloudMessage>("PointCloud");
    in_indices_ = modifier_->addInput<GenericVectorMessage, pcl::PointIndices>("Indices");
    output_ = modifier_->addOutput<PointCloudMessage>("Labeled PointCloud");
}

namespace implementation {

template<class PointT, class PointS>
struct Copy {
    static inline void apply(const PointS& src, PointT& dst)
    {
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
    }
};

template<>
struct Copy<pcl::PointXYZRGBL, pcl::PointXYZRGB> {
    inline static void apply(const pcl::PointXYZRGB& src, pcl::PointXYZRGBL& dst)
    {
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.r = src.r;
        dst.g = src.g;
        dst.b = src.b;
    }
};

template<class PointT, class PointS>
struct Impl {
    inline static void label(const typename pcl::PointCloud<PointT>::Ptr src,
                             typename pcl::PointCloud<PointS>::Ptr dst,
                             const LabelClusteredPointCloud::Indices &indices)
    {
        std::size_t n = src->points.size();
        dst->points.resize(n);
        for(std::size_t i = 0; i < n; ++i) {
            Copy<PointS, PointT>::apply(src->points[i], dst->points[i]);
            dst->points[i].label = 0;
        }

        int region = 1;
        for (LabelClusteredPointCloud::Indices::const_iterator it = indices.begin(); it != indices.end (); ++it) {
            for(std::vector<int>::const_iterator i = it->indices.begin(); i != it->indices.end(); ++i) {
                dst->points[*i].label = region;
            }
            ++region;
        }
    }
};

template<class PointT>
struct Label {
    static void apply(const typename pcl::PointCloud<PointT>::Ptr src,
                      PointCloudMessage::Ptr &dst_msg,
                      const LabelClusteredPointCloud::Indices &indices)
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
        Impl<PointT, pcl::PointXYZL>::label(src, cloud, indices);
        dst_msg->value = cloud;
    }
};

template<>
struct Label<pcl::PointXYZRGB> {
    static void apply(const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                      PointCloudMessage::Ptr &dst_msg,
                      const LabelClusteredPointCloud::Indices &indices)
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
        Impl<pcl::PointXYZRGB, pcl::PointXYZRGBL>::label(src, cloud, indices);
        dst_msg->value = cloud;
    }
};

template<>
struct Label<pcl::PointXY> {
    static void apply(const typename pcl::PointCloud<pcl::PointXY>::Ptr src,
                      PointCloudMessage::Ptr &dst_msg,
                      const LabelClusteredPointCloud::Indices &indices)
    {
        throw std::runtime_error("Pointcloud must be of type XYZ!");
    }
};
}

template <class PointT>
void LabelClusteredPointCloud::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id));

    implementation::Label<PointT>::apply(cloud, out, *cluster_indices);
    output_->publish(out);
}
