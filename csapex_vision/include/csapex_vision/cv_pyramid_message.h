#ifndef CV_PYRAMID_MESSAGE_H
#define CV_PYRAMID_MESSAGE_H

/// COMPONENT
#include <csapex_vision/encoding.h>

/// PROJECT
#include <csapex/model/message.h>

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

}
}


#endif // CV_PYRAMID_MESSAGE_H
