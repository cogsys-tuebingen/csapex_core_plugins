#ifndef CV_PYRAMID_MESSAGE_H
#define CV_PYRAMID_MESSAGE_H

/// COMPONENT
#include <csapex_opencv/encoding.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include <csapex/serialization/io/std_io.h>
#include <csapex_opencv/binary_io.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace YAML
{
template <typename T, typename S>
struct as_if;
}

namespace csapex
{
template <typename, typename, typename>
class ConverterTemplate;

namespace connection_types
{
struct CSAPEX_OPENCV_EXPORT CvPyramidMessage : public MessageTemplate<std::vector<cv::Mat>, CvPyramidMessage>
{
    friend class MessageTemplate<std::vector<cv::Mat>, CvPyramidMessage>;
    friend class YAML::as_if<CvPyramidMessage, void>;

    template <typename, typename, typename>
    friend class csapex::ConverterTemplate;

public:
    CvPyramidMessage(const Encoding& encoding);

    virtual void writeNative(const std::string& file, const std::string& base, const std::string& suffix) const;

    const Encoding& getEncoding() const;
    void setEncoding(const Encoding& e);

private:
    Encoding encoding;

private:
    CvPyramidMessage();
};

/// TRAITS
template <>
struct type<CvPyramidMessage>
{
    static std::string name()
    {
        return "std::vector<cv::Mat>";
    }
};
}  // namespace connection_types

template <>
inline std::shared_ptr<connection_types::CvPyramidMessage> makeEmpty<connection_types::CvPyramidMessage>()
{
    return std::shared_ptr<connection_types::CvPyramidMessage>(new connection_types::CvPyramidMessage(enc::bgr));
}

}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct convert<csapex::connection_types::CvPyramidMessage>
{
    static Node encode(const csapex::connection_types::CvPyramidMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::CvPyramidMessage& rhs);
};
}  // namespace YAML

#endif  // CV_PYRAMID_MESSAGE_H
