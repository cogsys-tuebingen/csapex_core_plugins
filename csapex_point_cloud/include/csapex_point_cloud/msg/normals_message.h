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
template<typename T, typename S>
struct as_if;
}

namespace csapex {
namespace connection_types {

struct NormalsMessage : public Message
{
    friend class YAML::as_if<NormalsMessage, void>;

    typedef std::shared_ptr<NormalsMessage> Ptr;
    typedef std::shared_ptr<NormalsMessage const> ConstPtr;

    NormalsMessage(const std::string& frame_id, Stamp stamp_micro_seconds);

    virtual TokenData::Ptr clone() const override;

    virtual TokenData::Ptr toType() const override;

    virtual std::string descriptiveName() const override;

    bool acceptsConnectionFrom(const TokenData* other_side) const override;

    pcl::PointCloud<pcl::Normal>::Ptr value;

private:
    NormalsMessage();
};


/// TRAITS
template <>
struct type<NormalsMessage> {
    static std::string name() {
        return "Normals";
    }
};

template <>
inline std::shared_ptr<NormalsMessage> makeEmpty<NormalsMessage>()
{
    return std::shared_ptr<NormalsMessage>(new NormalsMessage("/", 0));
}
}
}



/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::NormalsMessage> {
  static Node encode(const csapex::connection_types::NormalsMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::NormalsMessage& rhs);
};
}
#endif // NORMALS_MESSAGE_H
