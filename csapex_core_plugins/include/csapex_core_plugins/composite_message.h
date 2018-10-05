#ifndef COMPOSITE_MESSAGE_H
#define COMPOSITE_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/data_traits.hpp>
#include <csapex_core_plugins/csapex_core_lib_export.h>

/// SYSTEM
#include <boost/static_assert.hpp>
#include <string>
#include <vector>

namespace csapex
{
namespace connection_types
{
class CSAPEX_CORE_LIB_EXPORT CompositeMessage : public Message
{
protected:
    CLONABLE_IMPLEMENTATION(CompositeMessage);

public:
    typedef std::shared_ptr<CompositeMessage> Ptr;
    typedef std::shared_ptr<const CompositeMessage> ConstPtr;

    CompositeMessage(const std::string& frame_id = "/", Stamp stamp_micro_seconds = 0);

    TokenData::Ptr getSubType() const;

    template <typename T>
    static void registerType()
    {
        // not used for this type
    }

    template <typename T>
    static CompositeMessage::Ptr make()
    {
        return CompositeMessage::Ptr(new CompositeMessage(csapex::makeEmpty<T>(), "/", 0));
    }

    static CompositeMessage::Ptr make(TokenData::Ptr type)
    {
        return CompositeMessage::Ptr(new CompositeMessage(type->toType(), "/", 0));
    }

    static CompositeMessage::Ptr make();

    bool canConnectTo(const TokenData* other_side) const override;
    bool acceptsConnectionFrom(const TokenData* other_side) const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

private:
    CompositeMessage(TokenData::Ptr type, const std::string& frame_id, Stamp stamp_micro_seconds);

public:
    std::vector<TokenData::ConstPtr> value;

private:
    TokenData::Ptr type_;
};

template <>
struct type<CompositeMessage>
{
    static std::string name()
    {
        return "MessageComposite";
    }
};

}  // namespace connection_types
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct CSAPEX_EXPORT_PLUGIN convert<csapex::connection_types::CompositeMessage>
{
    static Node encode(const csapex::connection_types::CompositeMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::CompositeMessage& rhs);
};
}  // namespace YAML

#endif  // COMPOSITE_MESSAGE_H
