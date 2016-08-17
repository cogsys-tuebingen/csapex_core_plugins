#ifndef ROI_MESSAGE_H
#define ROI_MESSAGE_H

/// COMPONENT
#include <csapex_opencv/roi.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>

namespace csapex {
namespace connection_types {


struct CSAPEX_OPENCV_EXPORT RoiMessage : public MessageTemplate<Roi, RoiMessage>
{
    RoiMessage();
};


/// TRAITS
template <>
struct CSAPEX_OPENCV_EXPORT type<RoiMessage> {
    static std::string name() {
        return "ROI";
    }
};

}
}

/// YAML
namespace YAML {
template<>
struct CSAPEX_OPENCV_EXPORT convert<csapex::connection_types::RoiMessage> {
  static Node encode(const csapex::connection_types::RoiMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::RoiMessage& rhs);
};
}


#endif // ROI_MESSAGE_H
