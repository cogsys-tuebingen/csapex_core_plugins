#ifndef NORMALS_MESSAGE_H
#define NORMALS_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace csapex {
namespace connection_types {

struct NormalsMessage : public Message
{
    typedef std::shared_ptr<NormalsMessage> Ptr;
    typedef std::shared_ptr<NormalsMessage const> ConstPtr;

    NormalsMessage(const std::string& frame_id, Stamp stamp_micro_seconds);

    virtual ConnectionType::Ptr clone() const override;

    virtual ConnectionType::Ptr toType() const override;

    virtual std::string descriptiveName() const override;

    bool acceptsConnectionFrom(const ConnectionType* other_side) const override;

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