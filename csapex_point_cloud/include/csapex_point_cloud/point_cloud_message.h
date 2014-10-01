#ifndef POINT_CLOUD_MESSAGE_H
#define POINT_CLOUD_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/variant.hpp>
#include <boost/mpl/vector.hpp>

namespace csapex {
namespace connection_types {

/// ALL SUPPORTED POINT TYPES
typedef boost::mpl::vector<
pcl::PointXYZ,
pcl::PointXYZI,
pcl::PointXYZRGB,
pcl::PointXYZRGBL,
pcl::PointXYZL,
pcl::PointNormal
> PointCloudPointTypes;

namespace traits {
template <typename T> inline std::string name() { return ""; }

template <> inline std::string name<pcl::PointXYZ>() { return "PointXYZ"; }
template <> inline std::string name<pcl::PointXYZI>() { return "PointXYZI"; }
template <> inline std::string name<pcl::PointXYZRGB>() { return "PointXYZRGB"; }
template <> inline std::string name<pcl::PointXYZRGBL>() { return "PointXYZRGBL"; }
template <> inline std::string name<pcl::PointXYZL>() { return "PointXYZL"; }
template <> inline std::string name<pcl::PointNormal>() { return "PointNormal"; }
}

template<class T>
struct add_point_cloud_ptr
{
    typedef typename pcl::PointCloud<T>::Ptr type;
};

struct PointCloudMessage : public Message
{
    template <typename T>
    struct Dispatch : public boost::static_visitor<void>
    {
        Dispatch(T* pc, PointCloudMessage::Ptr msg)
            : pc_(pc), msg_(msg)
        {}

        template <typename PointCloudT>
        void operator () (PointCloudT cloud) const
        {
            typedef typename PointCloudT::element_type::PointType PointT;
            cloud->header.frame_id = msg_->frame_id;
            pc_->template inputCloud<PointT>(cloud);
        }

    private:
        T* pc_;
        PointCloudMessage::Ptr msg_;
    };

    typedef boost::shared_ptr<PointCloudMessage> Ptr;

    typedef typename boost::make_variant_over<
    typename boost::mpl::transform<
    PointCloudPointTypes, add_point_cloud_ptr<boost::mpl::_1>
    >::type
    >::type variant;

    PointCloudMessage(const std::string& frame_id, Stamp stamp);

    virtual ConnectionType::Ptr clone();

    virtual ConnectionType::Ptr toType();

    virtual std::string name() const;

    bool acceptsConnectionFrom(const ConnectionType* other_side) const;

    variant value;

private:
    PointCloudMessage();
};


/// TRAITS
template <>
struct type<PointCloudMessage> {
    static std::string name() {
        return "PointCloud";
    }
};

template <>
inline boost::shared_ptr<PointCloudMessage> makeEmpty<PointCloudMessage>()
{
    return boost::shared_ptr<PointCloudMessage>(new PointCloudMessage("/", 0));
}
}
}



/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::PointCloudMessage> {
  static Node encode(const csapex::connection_types::PointCloudMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::PointCloudMessage& rhs);
};
}
#endif // POINT_CLOUD_MESSAGE_H
