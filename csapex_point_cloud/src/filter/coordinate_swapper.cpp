/// HEADER
#include "coordinate_swapper.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <pcl/point_types.h>
#include <boost/assign/list_of.hpp>
#include <boost/utility/enable_if.hpp>

CSAPEX_REGISTER_CLASS(csapex::CoordinateSwapper, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace {

template <typename Any>
class PointTraits
{
    typedef char Small;
    class Big
    {
        char dummy[2];
    };

    template <typename Class> static Small testx(__typeof__(&Class::x)) ;
    template <typename Class> static Big testx(...);

    template <typename Class> static Small testy(__typeof__(&Class::y)) ;
    template <typename Class> static Big testy(...);

    template <typename Class> static Small testz(__typeof__(&Class::z)) ;
    template <typename Class> static Big testz(...);

public:
    enum { HasX = sizeof(testx<Any>(0)) == sizeof(Small) };
    enum { HasY = sizeof(testy<Any>(0)) == sizeof(Small) };
    enum { HasZ = sizeof(testz<Any>(0)) == sizeof(Small) };
    enum { HasXYZ = HasX && HasY &&  HasZ};
};

template<typename PointT>
inline static void swap(PointT &p1,
                       typename std::enable_if<PointTraits<PointT>::HasXYZ, PointT>::type* dummy = 0)
{
    PointT tmp = p1;

    p1.x = -tmp.y;
    p1.y = -tmp.z;
    p1.z =  tmp.x;
}

template<typename PointT>
struct Swapper {
    inline static void apply(const typename pcl::PointCloud<PointT>::ConstPtr &src,
                                   typename pcl::PointCloud<PointT>::Ptr &dst)
    {
        dst.reset(new pcl::PointCloud<PointT>(*src));

        PointT *dst_ptr = dst->points.data();
        int cols = src->width;
        int rows = src->height;
        for(int y = 0 ; y < rows ; ++y) {
            for(int x = 0 ; x < cols ; ++x) {
                int pos = y * cols + x;
                swap(dst_ptr[pos]);
            }
        }
    }


};
}

CoordinateSwapper::CoordinateSwapper()
{
}

void CoordinateSwapper::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    output_ = node_modifier.addOutput<PointCloudMessage>("Swapped Pointcloud");
}

void CoordinateSwapper::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));
    boost::apply_visitor (PointCloudMessage::Dispatch<CoordinateSwapper>(this, msg), msg->value);
}

template <class PointT>
void CoordinateSwapper::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_swapped;

    Swapper<PointT>::apply(cloud, cloud_swapped);

    PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
    out->value = cloud_swapped;
    msg::publish(output_, out);
}
