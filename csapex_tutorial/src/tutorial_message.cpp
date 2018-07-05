/// HEADER
#include <csapex_tutorial/tutorial_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::TutorialMessage)

using namespace csapex;
using namespace connection_types;

TutorialMessage::TutorialMessage()
    : MessageTemplate<bool, TutorialMessage>("/", 0)
{}


void TutorialMessage::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    Message::serialize(data, version);

    // in version 1 we saved data like this:
    // data << value;
    // but in version 2 we save the inverse
    version = SemanticVersion(2, 0, 0);
    data << !value;
}

void TutorialMessage::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Message::deserialize(data, version);

    if(version < SemanticVersion(2, 0, 0)) {
        // in version 1 we saved data like this:
        // data << value;
        data >> value;
    } else {
        // but in version 2 we save the inverse
        bool tmp;
        data >> tmp;
        value = !tmp;
    }
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::TutorialMessage>::encode(const csapex::connection_types::TutorialMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs, {2, 0, 0});
    node["value2"] = rhs.value;

    return node;
}

bool convert<csapex::connection_types::TutorialMessage>::decode(const Node& node, csapex::connection_types::TutorialMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    SemanticVersion version = convert<csapex::connection_types::Message>::decode(node, rhs);
    if(version < SemanticVersion{2, 0, 0}) {
        rhs.value = node["value"].as<bool>();
    } else {
        rhs.value = node["value2"].as<bool>();
    }

    return true;
}
}
