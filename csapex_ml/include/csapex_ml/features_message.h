#ifndef FEATURES_MESSAGE_H
#define FEATURES_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

namespace csapex {
namespace connection_types {


struct FeaturesMessage : public Message
{
    typedef boost::shared_ptr<FeaturesMessage> Ptr;

    FeaturesMessage();

    virtual ConnectionType::Ptr clone();
    virtual ConnectionType::Ptr toType();
    static ConnectionType::Ptr make();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);

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

#endif // FEATURES_MESSAGE_H
