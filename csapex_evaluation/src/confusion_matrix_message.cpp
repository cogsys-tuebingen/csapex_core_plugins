/// HEADER
#include <csapex_evaluation/confusion_matrix_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::ConfusionMatrixMessage)

using namespace csapex;
using namespace connection_types;

ConfusionMatrixMessage::ConfusionMatrixMessage(Message::Stamp stamp)
    : Message(type<ConfusionMatrixMessage>::name(), "/", stamp)
{}

Token::Ptr ConfusionMatrixMessage::clone() const
{
    Ptr new_msg(new ConfusionMatrixMessage);
    new_msg->confusion = confusion;
    return new_msg;
}

Token::Ptr ConfusionMatrixMessage::toType() const
{
    Ptr new_msg(new ConfusionMatrixMessage);
    return new_msg;
}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::ConfusionMatrixMessage>::encode(const csapex::connection_types::ConfusionMatrixMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    for(std::vector<int>::const_iterator class_it = rhs.confusion.classes.begin();
        class_it != rhs.confusion.classes.end(); ++class_it) {
        node["class_set"].push_back(*class_it);
    }

    for(std::vector<int>::const_iterator actual_it = rhs.confusion.classes.begin();
        actual_it != rhs.confusion.classes.end(); ++actual_it) {

        for(std::vector<int>::const_iterator predicted_it = rhs.confusion.classes.begin();
            predicted_it != rhs.confusion.classes.end(); ++predicted_it) {
            int value = 0;
            try {
                value = rhs.confusion.histogram.at(std::make_pair(*actual_it, *predicted_it));
            } catch(const std::exception& e) {
                value = 0;
            }

            node["matrix"][*actual_it][*predicted_it] = value;
        }
    }

    return node;
}

bool convert<csapex::connection_types::ConfusionMatrixMessage>::decode(const Node& node, csapex::connection_types::ConfusionMatrixMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);

    return true;
}
}
