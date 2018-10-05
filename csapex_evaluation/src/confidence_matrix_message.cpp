/// HEADER
#include <csapex_evaluation/confidence_matrix_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::ConfidenceMatrixMessage)

using namespace csapex;
using namespace connection_types;

ConfidenceMatrixMessage::ConfidenceMatrixMessage(Message::Stamp stamp) : Message(type<ConfidenceMatrixMessage>::name(), "/", stamp)
{
}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::ConfidenceMatrixMessage>::encode(const csapex::connection_types::ConfidenceMatrixMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    for (std::vector<int>::const_iterator class_it = rhs.confidence.classes.begin(); class_it != rhs.confidence.classes.end(); ++class_it) {
        node["class_set"].push_back(*class_it);
    }

    for (std::vector<int>::const_iterator actual_it = rhs.confidence.classes.begin(); actual_it != rhs.confidence.classes.end(); ++actual_it) {
        for (std::vector<int>::const_iterator predicted_it = rhs.confidence.classes.begin(); predicted_it != rhs.confidence.classes.end(); ++predicted_it) {
            float value = 0.f;
            int ups = 0;
            try {
                value = rhs.confidence.confidences.at(std::make_pair(*actual_it, *predicted_it));
                ups = rhs.confidence.confidence_updates.at(std::make_pair(*actual_it, *predicted_it));
            } catch (const std::exception& e) {
                value = 0.f;
                ups = 0;
            }

            node["matrix"][*actual_it][*predicted_it] = value;
            node["updates"][*actual_it][*predicted_it] = ups;
        }
    }

    return node;
}

bool convert<csapex::connection_types::ConfidenceMatrixMessage>::decode(const Node& node, csapex::connection_types::ConfidenceMatrixMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);

    return true;
}
}  // namespace YAML
