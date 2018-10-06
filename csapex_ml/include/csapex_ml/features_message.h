#ifndef FEATURES_MESSAGE_H
#define FEATURES_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include <csapex_ml/csapex_ml_export.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace connection_types
{
struct CSAPEX_ML_EXPORT FeaturesMessage : public Message
{
protected:
    CLONABLE_IMPLEMENTATION(FeaturesMessage);

public:
    enum class Type
    {
        CLASSIFICATION = 0,
        REGRESSION = 1
    };

    static const int INVALID_LABEL = -1;

    typedef std::shared_ptr<FeaturesMessage> Ptr;
    typedef std::shared_ptr<FeaturesMessage const> ConstPtr;

    FeaturesMessage(Type type, Message::Stamp stamp_micro_seconds = 0);
    FeaturesMessage(Message::Stamp stamp_micro_seconds = 0);

    std::vector<float> value;

    Type type;

    int classification;
    std::vector<float> regression_result;

    float confidence;
};

/// TRAITS
template <>
struct CSAPEX_ML_EXPORT type<FeaturesMessage>
{
    static std::string name()
    {
        return "FeaturesMessage";
    }
};
}  // namespace connection_types
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct CSAPEX_ML_EXPORT convert<csapex::connection_types::FeaturesMessage>
{
    static Node encode(const csapex::connection_types::FeaturesMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::FeaturesMessage& rhs);
};
}  // namespace YAML

#endif  // FEATURES_MESSAGE_H
