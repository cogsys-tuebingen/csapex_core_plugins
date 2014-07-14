/// HEADER
#include <csapex_point_cloud/point_cloud_message.h>

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <pcl/PCLPointField.h>
#include <pcl/io/boost.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/mpl/for_each.hpp>

using namespace csapex;
using namespace connection_types;

PointCloudMessage::PointCloudMessage(const std::string& frame_id)
    : Message (type<PointCloudMessage>::name(), frame_id)
{
}

ConnectionType::Ptr PointCloudMessage::clone() {
    Ptr new_msg(new PointCloudMessage(frame_id));
    new_msg->value = value;
    return new_msg;
}

ConnectionType::Ptr PointCloudMessage::toType() {
    Ptr new_msg(new PointCloudMessage("/"));
    return new_msg;
}

ConnectionType::Ptr PointCloudMessage::make(){
    Ptr new_msg(new PointCloudMessage("/"));
    return new_msg;
}


std::string PointCloudMessage::name() const
{
    return Message::name();
}

bool PointCloudMessage::acceptsConnectionFrom(const ConnectionType* other_side) const {
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

        YAML::Node points = node["points"];
        apex_assert(points.Type() == YAML::NodeType::Sequence);

        std::string file = "import_cloud.tmp";
        std::ofstream tmp(file.c_str());

        for(std::size_t i = 0; i < points.size(); ++i) {
            std::string line;
            points[i] >> line;
            tmp << line << '\n';
        }
        tmp.flush();
        tmp.close();

        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

        pcl::PCDReader reader;
        reader.read(file, *cloud);

        value = cloud;
    }

    const YAML::Node& node;
    PointCloudMessage::variant& value;
    std::string type;
};

struct Export : public boost::static_visitor<void> {
    Export(YAML::Emitter& yaml)
        : yaml(yaml)
    {}

    template <typename PointCloudT>
    void operator () (PointCloudT cloud_ptr) const
    {
        typedef typename PointCloudT::element_type::PointType PointT;

        yaml << YAML::Key << "point_type" << YAML::Value << traits::name<PointT>();

        pcl::PCDWriter writer;
        std::string file = "export_cloud.tmp";
        writer.writeASCII(file, *cloud_ptr);

        std::ifstream fi(file.c_str());
        std::string line;

        yaml << YAML::Key << "points"  << YAML::Value << YAML::BeginSeq;
        while(std::getline(fi, line)) {
            yaml << line;
        }
        yaml << YAML::EndSeq;
    }

    YAML::Emitter& yaml;
};
}

void PointCloudMessage::writeYaml(YAML::Emitter& yaml) const {
    boost::apply_visitor (Export(yaml), value);
}
void PointCloudMessage::readYaml(const YAML::Node& node) {
    if(!YAML::exists(node, "point_type")) {
        return;
    }
    if(!YAML::exists(node, "points")) {
        return;
    }

    std::string type;
    node["point_type"] >> type;

    Import converter(node, value, type);
    boost::mpl::for_each<connection_types::PointCloudPointTypes>( converter );
}

PointCloudMessage::PointCloudMessage()
    : Message (type<PointCloudMessage>::name(), "/")
{
}
