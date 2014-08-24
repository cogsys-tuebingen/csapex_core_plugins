#ifndef FEATURES_MESSAGE_H
#define FEATURES_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex {
namespace connection_types {


struct FeaturesMessage : public Message
{
    typedef boost::shared_ptr<FeaturesMessage> Ptr;

    FeaturesMessage();

    virtual ConnectionType::Ptr clone();
    virtual ConnectionType::Ptr toType();

    std::vector<float> value;
    int classification;
};

/// TRAITS
template <>
struct type<FeaturesMessage> {
    static std::string name() {
        return "Features";
    }
};
}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::FeaturesMessage> {
  static Node encode(const csapex::connection_types::FeaturesMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::FeaturesMessage& rhs);
};
}

#endif // FEATURES_MESSAGE_H
