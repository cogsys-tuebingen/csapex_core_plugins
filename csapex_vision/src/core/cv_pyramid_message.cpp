/// HEADER
#include <csapex_vision/cv_pyramid_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>
#include <csapex_vision/yaml_io.hpp>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::CvPyramidMessage)

using namespace csapex;
using namespace connection_types;


CvPyramidMessage::CvPyramidMessage()
    : encoding(enc::bgr)
{}

CvPyramidMessage::CvPyramidMessage(const Encoding& encoding)
    : encoding(encoding)
{}

ConnectionType::Ptr CvPyramidMessage::clone() const
{
    Ptr new_msg(new CvPyramidMessage(encoding));
    new_msg->value.assign(value.begin(), value.end());
    return new_msg;
}

void CvPyramidMessage::writeRaw(const std::string &path, const std::string &suffix) const
{
    for(unsigned int i = 0 ; i < value.size() ; ++i) {
        std::stringstream ss;
        ss << "_" << i;
        std::string file = path + "/img" + suffix + ss.str() + ".jpg";
        cv::imwrite(file, value.at(i));
    }
}

const Encoding& CvPyramidMessage::getEncoding() const
{
    return encoding;
}

void CvPyramidMessage::setEncoding(const Encoding &e)
{
    encoding = e;
}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::CvPyramidMessage>::encode(const csapex::connection_types::CvPyramidMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["value"] = rhs.value;
    node["encoding"] = rhs.getEncoding().toString();
    return node;
}

bool convert<csapex::connection_types::CvPyramidMessage>::decode(const Node& node, csapex::connection_types::CvPyramidMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    rhs.value = node["value"].as<cv::Mat>();
    rhs.setEncoding(Encoding::fromString(node["encoding"].as<std::string>()));
    return true;
}
}
