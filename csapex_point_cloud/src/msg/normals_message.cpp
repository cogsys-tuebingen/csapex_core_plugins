/// HEADER
#include <csapex_point_cloud/normals_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

/// SYSTEM
#include <pcl/PCLPointField.h>
#include <pcl/console/print.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::NormalsMessage)

using namespace csapex;
using namespace connection_types;

NormalsMessage::NormalsMessage(const std::string& frame_id, Message::Stamp stamp)
    : Message (type<NormalsMessage>::name(), frame_id, stamp)
{
}

NormalsMessage::NormalsMessage()
    : Message (type<NormalsMessage>::name(), "/", 0)
{
}


Token::Ptr NormalsMessage::clone() const
{
    Ptr new_msg(new NormalsMessage(frame_id, stamp_micro_seconds));
    new_msg->value = value;
    return new_msg;
}

Token::Ptr NormalsMessage::toType() const
{
    Ptr new_msg(new NormalsMessage("/", 0));
    return new_msg;
}

std::string NormalsMessage::descriptiveName() const
{
    return Message::descriptiveName();
}

bool NormalsMessage::acceptsConnectionFrom(const Token* other_side) const
{
    return dynamic_cast<const NormalsMessage*> (other_side);
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


Node convert<csapex::connection_types::NormalsMessage>::encode(const csapex::connection_types::NormalsMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    return node;
}

bool convert<csapex::connection_types::NormalsMessage>::decode(const Node& node, csapex::connection_types::NormalsMessage& rhs)
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

    return true;
}
}

