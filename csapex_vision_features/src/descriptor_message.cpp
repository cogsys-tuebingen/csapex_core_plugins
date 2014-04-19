/// HEADER
#include <csapex_vision_features/descriptor_message.h>

using namespace csapex;
using namespace connection_types;


DescriptorMessage::DescriptorMessage()
    : MessageTemplate<cv::Mat, DescriptorMessage> ("cv::Mat (descriptors)")
{}

bool DescriptorMessage::isBinary() const
{
    int d = value.depth();
    return d == CV_8U  || d == CV_8S || d == CV_16U || d == CV_16S || d == CV_32S;
}
