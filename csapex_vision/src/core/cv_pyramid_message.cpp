#include <csapex_vision/cv_pyramid_message.h>

using namespace csapex;
using namespace connection_types;


CvPyramidMessage::CvPyramidMessage()
    : encoding(enc::bgr)
{}

CvPyramidMessage::CvPyramidMessage(const Encoding& encoding)
    : encoding(encoding)
{}

ConnectionType::Ptr CvPyramidMessage::clone() {
    Ptr new_msg(new CvPyramidMessage(encoding));
    new_msg->value.assign(value.begin(), value.end());
    return new_msg;
}

void CvPyramidMessage::writeRaw(const std::string &path, const std::string &suffix) const
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
