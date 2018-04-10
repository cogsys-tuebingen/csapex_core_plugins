#ifndef DESCRIPTOR_MESSAGE_H
#define DESCRIPTOR_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include <csapex_opencv/binary_io.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
namespace connection_types {


struct DescriptorMessage : public MessageTemplate<cv::Mat, DescriptorMessage>
{
    DescriptorMessage();

    bool isBinary() const;

    void write(std::ostream &out) const;
};

/// TRAITS
template <>
struct type<DescriptorMessage> {
    static std::string name() {
        return "cv::Mat (descriptors)";
    }
};

}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::DescriptorMessage> {
  static Node encode(const csapex::connection_types::DescriptorMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::DescriptorMessage& rhs);
};
}


#endif // DESCRIPTOR_MESSAGE_H
