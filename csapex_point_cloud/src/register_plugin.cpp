/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_point_cloud/point_cloud_message.h>

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <sensor_msgs/PointCloud2.h>
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#pragma clang diagnostic ignored "-Wsign-compare"
#endif //__clang__
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/io.hpp>
#if __clang__
#pragma clang diagnostic pop
#endif //__clang__
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::RegisterPointCloudPlugin, csapex::CorePlugin)

using namespace csapex;


struct Sensor2Cloud
{
    struct Export : public boost::static_visitor<void> {
        Export(sensor_msgs::PointCloud2::Ptr &out)
            : out_(out)
        {}

        template <typename T>
        void operator () (T cloud) const
        {
            pcl::toROSMsg(*cloud, *out_);
        }

        sensor_msgs::PointCloud2::Ptr &out_;
    };

    struct try_convert
    {
        try_convert(const sensor_msgs::PointCloud2::ConstPtr &ros_msg, typename connection_types::PointCloudMessage::Ptr& out, bool full_match, bool& success)
            : ros_msg_(ros_msg), out_(out), full_match_(full_match), success_(success)
        {
            success_ = false;
        }

        template<typename PointT>
        void operator()(PointT& pt)
        {
            if(success_) {
                return;
            }

            std::vector<pcl::PCLPointField> fields;
            pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

            const sensor_msgs::PointCloud2::_fields_type& available_fields = ros_msg_->fields;


            if(full_match_) {
                if(fields.size() != available_fields.size()) {
                    return;
                }
            }

            for(size_t d = 0; d < fields.size (); ++d) {
                bool found = false;
                for(size_t f = 0; f < available_fields.size() && !found; ++f) {
                    if(fields[d].name == available_fields[f].name) {
                        found = true;
                    }
                }

                if(!found) {
                    return;
                }
            }

            success_ = true;
            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(*ros_msg_, *cloud);
            out_->value = cloud;
            out_->frame_id = ros_msg_->header.frame_id;
            out_->stamp_micro_seconds = ros_msg_->header.stamp.toNSec() / 1e3;
        }

        const sensor_msgs::PointCloud2::ConstPtr &ros_msg_;
        typename connection_types::PointCloudMessage::Ptr& out_;

        bool full_match_;
        bool& success_;
    };

    static connection_types::PointCloudMessage::Ptr ros2apex(const sensor_msgs::PointCloud2::ConstPtr &ros_msg) {
        u_int64_t stamp = ros_msg->header.stamp.toNSec() / 1e3;
        connection_types::PointCloudMessage::Ptr out(new connection_types::PointCloudMessage(ros_msg->header.frame_id, stamp));

        bool success;
        try_convert full_match_converter(ros_msg, out, true, success);
        boost::mpl::for_each<connection_types::PointCloudPointTypes>( full_match_converter );

        if(!success) {
            try_convert partial_converter(ros_msg, out, false, success);
            boost::mpl::for_each<connection_types::PointCloudPointTypes>( partial_converter );
        }

        if(!success) {
            std::cerr << "cannot convert message, type is not known. Fields:";
            for(const sensor_msgs::PointField& field : ros_msg->fields) {
                std::cerr << field.name << " ";
            }
            std::cerr << std::endl;
        }

        return out;
    }
    static sensor_msgs::PointCloud2::Ptr apex2ros(const typename connection_types::PointCloudMessage::ConstPtr& apex_msg) {
        sensor_msgs::PointCloud2::Ptr out(new sensor_msgs::PointCloud2);
        boost::apply_visitor (Export(out), apex_msg->value);
        out->header.frame_id = apex_msg->frame_id;
        out->header.stamp = out->header.stamp.fromNSec(apex_msg->stamp_micro_seconds * 1e3);
        return out;
    }
};



RegisterPointCloudPlugin::RegisterPointCloudPlugin()
{
}

void RegisterPointCloudPlugin::init(CsApexCore& core)
{
    Tag::createIfNotExists("PointCloud");

    RosMessageConversion::registerConversion<sensor_msgs::PointCloud2, connection_types::PointCloudMessage, Sensor2Cloud >();
}
