/// HEADER
#include <csapex_point_cloud/point_cloud_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

/// SYSTEM
#include <pcl/PCLPointField.h>
//#include <pcl/io/boost.h>
#include <pcl/console/print.h>
//#include <pcl/io/pcd_io.h>
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


ConnectionType::Ptr PointCloudMessage::clone() const
{
    Ptr new_msg(new PointCloudMessage(frame_id, stamp_micro_seconds));
    new_msg->value = value;
    return new_msg;
}

ConnectionType::Ptr PointCloudMessage::toType() const
{
    Ptr new_msg(new PointCloudMessage("/", 0));
    return new_msg;
}

std::string PointCloudMessage::descriptiveName() const
{
    return Message::descriptiveName();
}

bool PointCloudMessage::acceptsConnectionFrom(const ConnectionType* other_side) const
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

        const YAML::Node& data = node["data"];
        apex_assert(data.Type() == YAML::NodeType::Sequence);

//        std::string file = "import_cloud.tmp";
//        std::ofstream tmp(file.c_str());

//        for(std::size_t i = 0; i < data.size(); ++i) {
//            std::string line = data[i].as<std::string>();
//            tmp << line << '\n';
//        }
//        tmp.flush();
//        tmp.close();

//        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

//        pcl::PCDReader reader;
//        reader.read(file, *cloud);

//        value = cloud;
    }

    const YAML::Node& node;
    PointCloudMessage::variant& value;
    std::string type;
};

struct Export : public boost::static_visitor<void> {
    Export(YAML::Node& node)
        : node(node)
    {}

    template <typename PointCloudT>
    void operator () (PointCloudT cloud_ptr) const
    {
        typedef typename PointCloudT::element_type::PointType PointT;

        node["point_type"] = traits::name<PointT>();

//        pcl::PCDWriter writer;
//        std::string file = "export_cloud.tmp";
//        writer.writeASCII(file, *cloud_ptr);

//        std::ifstream fi(file.c_str());
//        std::string line;

//        while(std::getline(fi, line)) {
//            node["data"].push_back(line);
//        }
    }

    YAML::Node& node;
};
}



/// YAML
namespace YAML {
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
