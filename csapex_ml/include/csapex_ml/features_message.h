#ifndef FEATURES_MESSAGE_H
#define FEATURES_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

namespace csapex {
namespace connection_types {


struct FeaturesMessage : public MessageTemplate<std::vector<float>, FeaturesMessage>
{
    FeaturesMessage();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);
};

}
}

#endif // FEATURES_MESSAGE_H
