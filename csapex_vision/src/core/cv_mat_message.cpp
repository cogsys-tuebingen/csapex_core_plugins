/// HEADER
#include <csapex_vision/cv_mat_message.h>

/// COMPONENT
#include <csapex_vision/yaml_io.hpp>
#include <csapex/utility/register_msg.h>

using namespace csapex;
using namespace connection_types;

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::CvMatMessage)

CvMatMessage::CvMatMessage()
    : MessageTemplate<cv::Mat, CvMatMessage> ("/camera"), encoding(enc::bgr)
{}

CvMatMessage::CvMatMessage(const Encoding& encoding)
    : MessageTemplate<cv::Mat, CvMatMessage> ("/camera"), encoding(encoding)
{}

ConnectionType::Ptr CvMatMessage::clone() {
    Ptr new_msg(new CvMatMessage(encoding));
    value.copyTo(new_msg->value);
    return new_msg;
}

void CvMatMessage::writeRaw(const std::string &path, const std::string &suffix) const
{
    std::string file = path + "/img" + suffix + ".jpg";
    cv::imwrite(file, value);
}

const Encoding& CvMatMessage::getEncoding() const
{
    return encoding;
}

void CvMatMessage::setEncoding(const Encoding &e)
{
    encoding = e;
}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::CvMatMessage>::encode(const csapex::connection_types::CvMatMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["value"] = rhs.value;
    node["encoding"] = rhs.getEncoding().toString();
    return node;
}

bool convert<csapex::connection_types::CvMatMessage>::decode(const Node& node, csapex::connection_types::CvMatMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    // TODO: ENCODING! ALSO IN CvPyramidMessage
    // rhs.encoding = node["encoding"].as<>();
    rhs.value = node["value"].as<cv::Mat>();
    return true;
}
}
