#include <csapex_vision/cv_pyramid_message.h>

using namespace csapex;
using namespace connection_types;


CvPyramidMessage::CvPyramidMessage()
    : MessageTemplate<std::vector<cv::Mat>, CvPyramidMessage> ("std::vector<cv::Mat>"), encoding(enc::bgr)
{}

CvPyramidMessage::CvPyramidMessage(const Encoding& encoding)
    : MessageTemplate<std::vector<cv::Mat>, CvPyramidMessage> ("std::vector<cv::Mat>"), encoding(encoding)
{}

ConnectionType::Ptr CvPyramidMessage::clone() {
    Ptr new_msg(new CvPyramidMessage(encoding));
    new_msg->value.assign(value.begin(), value.end());
    return new_msg;
}

void CvPyramidMessage::writeRaw(const std::string &path, const std::string &suffix)
{
    for(unsigned int i = 0 ; i < value.size() ; ++i) {
        std::stringstream ss;
        ss << "_" << i;
        std::string file = path + "/img" + suffix + ss.str() + ".jpg";
        cv::imwrite(file, value.at(i));
    }
}

const Encoding& CvPyramidMessage::getEncoding() const
{
    return encoding;
}

void CvPyramidMessage::setEncoding(const Encoding &e)
{
    encoding = e;
}
