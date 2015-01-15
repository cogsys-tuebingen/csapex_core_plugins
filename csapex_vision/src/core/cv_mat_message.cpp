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

CvMatMessage::CvMatMessage(const Encoding& encoding, Message::Stamp stamp)
    : MessageTemplate<cv::Mat, CvMatMessage> ("/camera", stamp), encoding(encoding)
{}

ConnectionType::Ptr CvMatMessage::clone() const
{
    Ptr new_msg(new CvMatMessage(encoding, stamp_micro_seconds));
    value.copyTo(new_msg->value);
    new_msg->frame_id = frame_id;
    return new_msg;
}

void CvMatMessage::writeRaw(const std::string &path, const std::string &suffix) const
{
    std::string file = path + "/img" + suffix + ".png";
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

bool CvMatMessage::hasChannels(std::size_t count) const
{
    return value.channels() == (int) count && encoding.channelCount() == count;
}


bool CvMatMessage::hasChannels(std::size_t count, int mat_type) const
{
    return value.channels() == (int) count && encoding.channelCount() == count && value.type() == mat_type;
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
    rhs.value = node["value"].as<cv::Mat>();
    rhs.setEncoding(Encoding::fromString(node["encoding"].as<std::string>()));
    return true;
}
}
