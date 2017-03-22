/// HEADER
#include <csapex_point_cloud/msg/point_cloud_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

/// SYSTEM
#include <pcl/PCLPointField.h>
#include <pcl/console/print.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::PointCloudMessage)

using namespace csapex;
using namespace connection_types;

PointCloudMessage::PointCloudMessage(const std::string& frame_id, Message::Stamp stamp)
    : Message (type<PointCloudMessage>::name(), frame_id, stamp)
{
}

PointCloudMessage::PointCloudMessage()
    : Message (type<PointCloudMessage>::name(), "/", 0)
{
}


TokenData::Ptr PointCloudMessage::clone() const
{
    Ptr new_msg(new PointCloudMessage(frame_id, stamp_micro_seconds));
    new_msg->value = value;
    return new_msg;
}

TokenData::Ptr PointCloudMessage::toType() const
{
    Ptr new_msg(new PointCloudMessage("/", 0));
    return new_msg;
}

std::string PointCloudMessage::descriptiveName() const
{
    return Message::descriptiveName();
}

bool PointCloudMessage::acceptsConnectionFrom(const TokenData* other_side) const
{
    return dynamic_cast<const PointCloudMessage*> (other_side);
}



namespace {
struct Import  {
    Import(const YAML::Node& node, PointCloudMessage::variant& value, const std::string& type)
        : node(node), value(value), type(type)
    {}

    template <typename PointT>
    void operator () (PointT) const
    {
        if(traits::name<PointT>() != type) {
            return;
        }

        pcl::PCLPointCloud2 pcl_pc;


        pcl_pc.header = node["header"].as<pcl::PCLHeader>();
        pcl_pc.height = node["height"].as<pcl::uint32_t>();
        pcl_pc.width = node["width"].as<pcl::uint32_t>();
        pcl_pc.fields = node["fields"].as<std::vector< ::pcl::PCLPointField>>();
        pcl_pc.is_bigendian = node["is_bigendian"].as<pcl::uint8_t>();
        pcl_pc.point_step = node["point_step"].as<pcl::uint32_t>();
        pcl_pc.row_step = node["row_step"].as<pcl::uint32_t>();

        YAML::Binary data = node["data"].as<YAML::Binary>();
        data.swap(pcl_pc.data);

        pcl_pc.is_dense = node["is_dense"].as<pcl::uint8_t>();

        boost::shared_ptr<pcl::PointCloud<PointT>> cloud(new pcl::PointCloud<PointT>);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);
        value = cloud;
    }

    const YAML::Node& node;
    PointCloudMessage::variant& value;
    std::string type;
};

template <typename PointT>
void write(std::stringstream& ss, const PointT& pt)
{
    ss << pt.x << "," << pt.y << ", " << pt.z << ", ";
}


template <>
void write<pcl::PointXYZI>(std::stringstream& ss, const pcl::PointXYZI& pt)
{
    ss << pt.x << "," << pt.y << ", " << pt.z << ", " << pt.intensity << ", ";
}
struct Export : public boost::static_visitor<void> {
    Export(YAML::Node& node)
        : node(node)
    {}

    template <typename PointCloudT>
    void operator () (PointCloudT cloud_ptr) const
    {
        typedef typename PointCloudT::element_type::PointType PointT;

        node["point_type"] = traits::name<PointT>();

        pcl::PCLPointCloud2 pcl_pc;
        pcl::toPCLPointCloud2(*cloud_ptr, pcl_pc);

        node["header"] = pcl_pc.header;
        node["height"] = pcl_pc.height;
        node["width"] = pcl_pc.width;
        node["fields"] = pcl_pc.fields;
        node["is_bigendian"] = pcl_pc.is_bigendian;
        node["point_step"] = pcl_pc.point_step;
        node["row_step"] = pcl_pc.row_step;
        //node["data"] = YAML::Binary(pcl_pc.data.data(), pcl_pc.data.size());
        std::stringstream ss;
        for(const PointT& point : cloud_ptr->points) {
            write(ss, point);
            node["data"] = ss.str();
        }
        node["is_dense"] = pcl_pc.is_dense;
    }

    YAML::Node& node;
};
}



/// YAML
namespace YAML {

template<>
struct convert<pcl::PCLHeader> {
  static Node encode(const pcl::PCLHeader& rhs)
  {
      Node n;
      n["seq"] = rhs.seq;
      n["stamp"] = rhs.stamp;
      n["frame_id"] = rhs.frame_id;
      return n;
  }

  static bool decode(const Node& node, pcl::PCLHeader& rhs)
  {
      rhs.seq = node["seq"].as<pcl::uint32_t>();
      rhs.stamp = node["stamp"].as<pcl::uint64_t>();
      rhs.frame_id = node["frame_id"].as<std::string>();
      return true;
  }
};

template<>
struct convert<pcl::PCLPointField> {
  static Node encode(const pcl::PCLPointField& rhs)
  {
      Node n;
      n["name"] = rhs.name;
      n["offset"] = rhs.offset;
      n["datatype"] = rhs.datatype;
      n["count"] = rhs.count;
      return n;
  }

  static bool decode(const Node& node, pcl::PCLPointField& rhs)
  {
      rhs.name = node["name"].as<std::string>();
      rhs.offset = node["offset"].as<pcl::uint32_t>();
      rhs.datatype = node["datatype"].as<pcl::uint8_t>();
      rhs.count = node["count"].as<pcl::uint32_t>();
      return true;
  }
};


Node convert<csapex::connection_types::PointCloudMessage>::encode(const csapex::connection_types::PointCloudMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    boost::apply_visitor (Export(node), rhs.value);
    return node;
}

bool convert<csapex::connection_types::PointCloudMessage>::decode(const Node& node, csapex::connection_types::PointCloudMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    if(!node["point_type"].IsDefined()) {
        return false;
    }
    if(!node["data"].IsDefined()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);

    std::string type = node["point_type"].as<std::string>();

    Import converter(node, rhs.value, type);
    boost::mpl::for_each<connection_types::PointCloudPointTypes>( converter );

    return true;
}
}
