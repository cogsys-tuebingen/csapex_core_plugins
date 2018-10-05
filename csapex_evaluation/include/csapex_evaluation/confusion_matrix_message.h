#ifndef CONFUSION_MATRIX_MESSAGE_H
#define CONFUSION_MATRIX_MESSAGE_H

/// COMPONENT
#include <csapex_evaluation/confusion_matrix.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace connection_types
{
struct ConfusionMatrixMessage : public Message
{
protected:
    CLONABLE_IMPLEMENTATION(ConfusionMatrixMessage);

public:
    typedef std::shared_ptr<ConfusionMatrixMessage> Ptr;
    typedef std::shared_ptr<ConfusionMatrixMessage const> ConstPtr;

    ConfusionMatrixMessage(Message::Stamp stamp_micro_seconds = 0);

    ConfusionMatrix confusion;
};

/// TRAITS
template <>
struct type<ConfusionMatrixMessage>
{
    static std::string name()
    {
        return "ConfusionMatrixMessage";
    }
};
}  // namespace connection_types
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct convert<csapex::connection_types::ConfusionMatrixMessage>
{
    static Node encode(const csapex::connection_types::ConfusionMatrixMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::ConfusionMatrixMessage& rhs);
};
}  // namespace YAML

#endif  // CONFUSION_MATRIX_MESSAGE_H
