#ifndef NORMALS_MESSAGE_H
#define NORMALS_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/msg/token_traits.h>

/// SYSTEM
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace YAML
{
template <typename T, typename S>
struct as_if;
}

namespace csapex
{
namespace connection_types
{
struct NormalsMessage : public Message
{
protected:
    CLONABLE_IMPLEMENTATION(NormalsMessage);

public:
    friend class YAML::as_if<NormalsMessage, void>;

    typedef std::shared_ptr<NormalsMessage> Ptr;
    typedef std::shared_ptr<NormalsMessage const> ConstPtr;

    NormalsMessage(const std::string& frame_id, Stamp stamp_micro_seconds);

    virtual std::string descriptiveName() const override;

    bool acceptsConnectionFrom(const TokenData* other_side) const override;

    pcl::PointCloud<pcl::Normal>::Ptr value;

private:
    NormalsMessage();
};

/// TRAITS
template <>
struct type<NormalsMessage>
{
    static std::string name()
    {
        return "Normals";
    }
};

}  // namespace connection_types

template <>
inline std::shared_ptr<connection_types::NormalsMessage> makeEmpty<connection_types::NormalsMessage>()
{
    return std::shared_ptr<connection_types::NormalsMessage>(new connection_types::NormalsMessage("/", 0));
}
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct convert<csapex::connection_types::NormalsMessage>
{
    static Node encode(const csapex::connection_types::NormalsMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::NormalsMessage& rhs);
};
}  // namespace YAML
#endif  // NORMALS_MESSAGE_H
