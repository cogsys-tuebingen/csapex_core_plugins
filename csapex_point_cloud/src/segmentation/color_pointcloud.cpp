/// HEADER
#include "color_pointcloud.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <pcl/point_types.h>
#include <pcl/conversions.h>


CSAPEX_REGISTER_CLASS(csapex::ColorPointCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

#define FLOOD_DEFAULT_LABEL 0

ColorPointCloud::ColorPointCloud()
{
}

void ColorPointCloud::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

    boost::apply_visitor (PointCloudMessage::Dispatch<ColorPointCloud>(this, msg), msg->value);
}

void ColorPointCloud::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<PointCloudMessage>("Labeled PointCloud");
    output_ = node_modifier.addOutput<PointCloudMessage>("Colored PointCloud");
}

namespace implementation {

struct Color {
    Color(uchar _r = 0, uchar _g = 0, uchar _b = 0) :
        r(_r),
        g(_g),
        b(_b){}

    uchar r;
    uchar g;
    uchar b;
};

template<class PointT>
struct Impl {
    inline static void convert(const typename pcl::PointCloud<PointT>::ConstPtr& src,
                               typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst)
    {
        std::map<unsigned int, Color> colors;
        colors.insert(std::make_pair(FLOOD_DEFAULT_LABEL, Color()));
        for(typename pcl::PointCloud<PointT>::const_iterator it = src->begin() ; it != src->end() ; ++it) {
            if(colors.find(it->label) == colors.end()) {
                double r = 0.0, g = 0.0, b = 0.0;
                color::fromCount(it->label+1, r,g,b);
                colors.insert(std::make_pair(it->label,Color(r,g,b)));
            }
            Color c = colors.at(it->label);
            pcl::PointXYZRGB p(c.r, c.g, c.b);
            p.x = it->x;
            p.y = it->y;
            p.z = it->z;
            dst->push_back(p);
        }
    }
};

template<class PointT>
struct Conversion {
    static void apply(const typename pcl::PointCloud<PointT>::ConstPtr& src,
                      typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst)
    {
        throw std::runtime_error("Type of pointcloud must be labeled!");
    }
};

template<>
struct Conversion<pcl::PointXYZL>{
    static void apply(const typename pcl::PointCloud<pcl::PointXYZL>::ConstPtr& src,
                      typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst)
    {
        Impl<pcl::PointXYZL>::convert(src, dst);
    }

};

template<>
struct Conversion<pcl::PointXYZRGBL>{
    static void apply(const typename pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& src,
                      typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst)
    {
        Impl<pcl::PointXYZRGBL>::convert(src, dst);
    }
};
}

template <class PointT>
void ColorPointCloud::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    implementation::Conversion<PointT>::apply(cloud, out_cloud);

    out_cloud->height = cloud->height;
    out_cloud->header = cloud->header;
    out_cloud->width  = cloud->width;
    out_cloud->is_dense = cloud->is_dense;

    out->value = out_cloud;
    msg::publish(output_, out);
}
