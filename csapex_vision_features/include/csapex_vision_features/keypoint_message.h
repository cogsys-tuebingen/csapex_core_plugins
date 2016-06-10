#ifndef KEYPOINT_MESSAGE_H
#define KEYPOINT_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
namespace connection_types {


struct KeypointMessage : public MessageTemplate<std::vector<cv::KeyPoint>, KeypointMessage>
{
    KeypointMessage();

    bool isContainer() const override;
    TokenData::Ptr nestedType() const override;
    virtual void addNestedValue(const TokenData::ConstPtr &msg) override;
    virtual TokenData::ConstPtr nestedValue(std::size_t i) const override;
    virtual std::size_t nestedValueCount() const override;
};


/// TRAITS
template <>
struct type<KeypointMessage> {
    static std::string name() {
        return "std::vector<cv::KeyPoint>";
    }
};

}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::KeypointMessage> {
  static Node encode(const csapex::connection_types::KeypointMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::KeypointMessage& rhs);
};
}


#endif // KEYPOINT_MESSAGE_H
