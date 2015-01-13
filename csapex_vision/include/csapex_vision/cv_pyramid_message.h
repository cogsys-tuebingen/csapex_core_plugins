#ifndef CV_PYRAMID_MESSAGE_H
#define CV_PYRAMID_MESSAGE_H

/// COMPONENT
#include <csapex_vision/encoding.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {

template <typename,typename,typename> class ConverterTemplate;

namespace connection_types {

struct CvPyramidMessage : public MessageTemplate<std::vector<cv::Mat>, CvPyramidMessage>
{
    friend class MessageTemplate<std::vector<cv::Mat>, CvPyramidMessage>;

    template <typename,typename,typename> friend class csapex::ConverterTemplate;

public:
    CvPyramidMessage(const Encoding& encoding);
    virtual ConnectionType::Ptr clone();

    virtual void writeRaw(const std::string &file, const std::string &suffix) const;

    const Encoding &getEncoding() const;
    void setEncoding(const Encoding& e);

private:
    Encoding encoding;

private:
    CvPyramidMessage();
};


/// TRAITS
template <>
struct type<CvPyramidMessage> {
    static std::string name() {
        return "std::vector<cv::Mat>";
    }
};

template <>
inline std::shared_ptr<CvPyramidMessage> makeEmpty<CvPyramidMessage>()
{
    return std::shared_ptr<CvPyramidMessage>(new CvPyramidMessage(enc::bgr));
}

}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::CvPyramidMessage> {
  static Node encode(const csapex::connection_types::CvPyramidMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::CvPyramidMessage& rhs);
};
}


#endif // CV_PYRAMID_MESSAGE_H
