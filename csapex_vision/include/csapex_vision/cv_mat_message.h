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
    CvMatMessage(const Encoding& encoding);
    virtual ConnectionType::Ptr clone();

    virtual void writeRaw(const std::string &file, const std::string &suffix) const;

    const Encoding &getEncoding() const;
    void setEncoding(const Encoding& e);

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

}
}

#endif // CV_MAT_MESSAGE_H
