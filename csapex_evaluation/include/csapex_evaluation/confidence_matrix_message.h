#ifndef CONFIDENCE_MATRIX_MESSAGE_H
#define CONFIDENCE_MATRIX_MESSAGE_H

/// COMPONENT
#include <csapex_evaluation/confidence_matrix.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex {
namespace connection_types {


struct ConfidenceMatrixMessage : public Message
{    
protected:
    CLONABLE_IMPLEMENTATION(ConfidenceMatrixMessage);

public:
    typedef std::shared_ptr<ConfidenceMatrixMessage> Ptr;
    typedef std::shared_ptr<ConfidenceMatrixMessage const> ConstPtr;

    ConfidenceMatrixMessage(Message::Stamp stamp_micro_seconds = 0);

    ConfidenceMatrix confidence;
};

/// TRAITS
template <>
struct type<ConfidenceMatrixMessage> {
    static std::string name() {
        return "ConfidenceMatrixMessage";
    }
};
}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::ConfidenceMatrixMessage> {
  static Node encode(const csapex::connection_types::ConfidenceMatrixMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::ConfidenceMatrixMessage& rhs);
};
}

#endif // CONFIDENCE_MATRIX_MESSAGE_H
