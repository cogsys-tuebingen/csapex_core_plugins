/// HEADER
#include <csapex_vision_features/descriptor_message.h>

/// PROJECT
#include <csapex_vision/yaml_io.hpp>

using namespace csapex;
using namespace connection_types;


DescriptorMessage::DescriptorMessage()
    : MessageTemplate<cv::Mat, DescriptorMessage> ("/")
{}

bool DescriptorMessage::isBinary() const
{
    int d = value.depth();
    return d == CV_8U  || d == CV_8S || d == CV_16U || d == CV_16S || d == CV_32S;
}

namespace {
std::string typeToString(const int type)
{
    switch(type & 7) {
    case CV_8U:
        return "CV_8U";
    case CV_16U:
        return "CV_16U";
    case CV_8S:
        return "CV_8S";
    case CV_16S:
        return "CV_16S";
    case CV_32S:
        return "CV_32S";
    case CV_32F:
        return "CV_32F";
    case CV_64F:
        return "CV_64F";
    default:
        return "";
    }
}
}


void DescriptorMessage::write(std::ostream &out) const
{
    out << "Dimension: " << value.rows << " x " << value.cols;
    out << "  Type: " << typeToString(value.type()) << "C" << value.channels();
    out << "  Data: " << value;
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::DescriptorMessage>::encode(const csapex::connection_types::DescriptorMessage& rhs)
{
    Node node;

    node["type"] = typeToString(rhs.value.type());
    node["value"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::DescriptorMessage>::decode(const Node& node, csapex::connection_types::DescriptorMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    rhs.value = node["value"].as<cv::Mat>();
    return true;
}
}

