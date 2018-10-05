#ifndef CV_MAT_MESSAGE_H
#define CV_MAT_MESSAGE_H

/// COMPONENT
#include <csapex_opencv/encoding.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>
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
struct CSAPEX_OPENCV_EXPORT CvMatMessage : public MessageTemplate<cv::Mat, CvMatMessage>
{
    friend class MessageTemplate<cv::Mat, CvMatMessage>;
    friend class YAML::as_if<CvMatMessage, void>;

    template <typename, typename, typename>
    friend class csapex::ConverterTemplate;

public:
    CvMatMessage(const Encoding& encoding, const std::string& frame_id, Stamp stamp_micro_seconds);
    ~CvMatMessage();

    virtual void writeNative(const std::string& file, const std::string& base, const std::string& suffix) const override;

    const Encoding& getEncoding() const;
    void setEncoding(const Encoding& e);

    bool hasChannels(std::size_t count) const;
    bool hasChannels(std::size_t count, int mat_type) const;

    bool cloneData(const CvMatMessage& other);

private:
    Encoding encoding;

private:
    CvMatMessage();
};

/// TRAITS
// stringification
template <>
struct CSAPEX_OPENCV_EXPORT type<CvMatMessage>
{
    static std::string name()
    {
        return "cv::Mat";
    }
};

// accessor to raw data
template <>
struct CSAPEX_OPENCV_EXPORT MessageContainer<cv::Mat, false>
{
    typedef CvMatMessage type;

    static cv::Mat& access(CvMatMessage& msg)
    {
        return msg.value;
    }
    static const cv::Mat& accessConst(const CvMatMessage& msg)
    {
        return msg.value;
    }
};

}  // namespace connection_types

// constructor for empty image
template <>
inline CSAPEX_OPENCV_EXPORT std::shared_ptr<connection_types::CvMatMessage> makeEmpty<connection_types::CvMatMessage>()
{
    return std::shared_ptr<connection_types::CvMatMessage>(new connection_types::CvMatMessage(enc::bgr, "camera", 0));
}

}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct CSAPEX_OPENCV_EXPORT convert<csapex::connection_types::CvMatMessage>
{
    static Node encode(const csapex::connection_types::CvMatMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::CvMatMessage& rhs);
};
}  // namespace YAML

#endif  // CV_MAT_MESSAGE_H
