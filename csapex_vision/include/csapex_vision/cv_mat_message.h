#ifndef CV_MAT_MESSAGE_H
#define CV_MAT_MESSAGE_H

/// COMPONENT
#include <csapex_vision/encoding.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {

template <typename,typename,typename> class ConverterTemplate;

namespace connection_types {

struct CvMatMessage : public MessageTemplate<cv::Mat, CvMatMessage>
{
    friend class MessageTemplate<cv::Mat, CvMatMessage>;

    template <typename,typename,typename> friend class csapex::ConverterTemplate;

public:
    CvMatMessage(const Encoding& encoding, Stamp stamp_micro_seconds);
    virtual ConnectionType::Ptr clone() const override;

    virtual void writeRaw(const std::string &file, const std::string &base, const std::string &suffix) const override;

    const Encoding &getEncoding() const;
    void setEncoding(const Encoding& e);

    bool hasChannels(std::size_t count) const;
    bool hasChannels(std::size_t count, int mat_type) const;

private:
    Encoding encoding;

private:
    CvMatMessage();
};


/// TRAITS
template <>
struct type<CvMatMessage> {
    static std::string name() {
        return "cv::Mat";
    }
};

template <>
inline std::shared_ptr<CvMatMessage> makeEmpty<CvMatMessage>()
{
    return std::shared_ptr<CvMatMessage>(new CvMatMessage(enc::bgr, 0));
}

}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::CvMatMessage> {
  static Node encode(const csapex::connection_types::CvMatMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::CvMatMessage& rhs);
};
}

#endif // CV_MAT_MESSAGE_H
