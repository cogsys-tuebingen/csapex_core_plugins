#ifndef FEATURES_MESSAGE_H
#define FEATURES_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

namespace csapex {
namespace connection_types {


struct FeaturesMessage : public MessageTemplate<std::vector<float>, FeaturesMessage>
{
    FeaturesMessage();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);
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

#endif // FEATURES_MESSAGE_H
