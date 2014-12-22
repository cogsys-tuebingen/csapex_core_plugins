/// HEADER
#include <csapex_vision/image_renderer.h>

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <utils_qt/QtCvImageConverter.h>

CSAPEX_REGISTER_CLASS(csapex::ImageRenderer, csapex::MessageRenderer)

using namespace csapex;


QSharedPointer<QImage> ImageRenderer::doRender(const connection_types::CvMatMessage &msg)
{
    Encoding encoding = msg.getEncoding();

    if(encoding.matches(enc::rgb)) {
        cv::Mat mat = msg.value;
        return QSharedPointer<QImage>(new QImage((uchar*) mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888));

    } else if(encoding.matches(enc::bgr) || encoding.matches(enc::mono)) {
        return QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(msg.value);
    }

    return QSharedPointer<QImage>();
}
