/// HEADER
#include <csapex_opencv/cv_mat_message.h>

/// COMPONENT
#include <csapex_opencv/yaml_io.hpp>
#include <csapex/utility/register_msg.h>

using namespace csapex;
using namespace connection_types;

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::CvMatMessage)


CvMatMessage::CvMatMessage()
    : MessageTemplate<cv::Mat, CvMatMessage> ("camera"), encoding(enc::bgr)
{
}

CvMatMessage::CvMatMessage(const Encoding& encoding, const std::string &frame_id, Message::Stamp stamp)
    : MessageTemplate<cv::Mat, CvMatMessage> (frame_id, stamp), encoding(encoding)
{
}

CvMatMessage::~CvMatMessage()
{
}

void CvMatMessage::cloneData(const CvMatMessage& other)
{
    Message::cloneDataFrom(other);

    encoding = other.encoding;
    value = other.value.clone();
}

void CvMatMessage::writeNative(const std::string &path, const std::string &base, const std::string &suffix) const
{
    std::string file = path + "/" + base + "_" + suffix + ".png";
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
CSAPEX_EXPORT_PLUGIN Node convert<csapex::connection_types::CvMatMessage>::encode(const csapex::connection_types::CvMatMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs, {0, 0, 0});
    node["value"] = rhs.value;
    node["encoding"] = rhs.getEncoding().toString();
    return node;
}

CSAPEX_EXPORT_PLUGIN bool convert<csapex::connection_types::CvMatMessage>::decode(const Node& node, csapex::connection_types::CvMatMessage& rhs)
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
