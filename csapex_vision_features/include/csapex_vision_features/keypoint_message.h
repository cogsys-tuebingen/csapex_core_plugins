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

#endif // KEYPOINT_MESSAGE_H
