/// HEADER
#include <csapex_vision/image_renderer.h>

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/QtCvImageConverter.h>

/// SYSTEM
#include <QImage>
#include <QPainter>

CSAPEX_REGISTER_CLASS(csapex::ImageRenderer, csapex::MessageRenderer)

using namespace csapex;

namespace
{
struct mat_lifetime_extender
{
    mat_lifetime_extender(const cv::Mat& mat)
        : mat(mat)
    {}

    cv::Mat mat;
};
void clean_mat(void* mat)
{
    delete static_cast<mat_lifetime_extender*>(mat);
}

}

QImage ImageRenderer::doRender(const connection_types::CvMatMessage &msg)
{
    Encoding encoding = msg.getEncoding();

    if(encoding.matches(enc::rgb)) {
        cv::Mat mat = msg.value;
        return QImage((uchar*) mat.data,
                      mat.cols, mat.rows, mat.step,
                      QImage::Format_RGB888,
                      clean_mat, new mat_lifetime_extender(mat));

    } else if(encoding.matches(enc::bgr) || encoding.matches(enc::mono)) {
        return QtCvImageConverter::Converter<QImage>::mat2QImage(msg.value);

    } else if(encoding.matches(enc::depth)) {
        return QtCvImageConverter::Converter<QImage>::mat2QImage(msg.value);

    } else if(msg.value.channels() == 1 || msg.value.channels() == 3) {
        return QtCvImageConverter::Converter<QImage>::mat2QImage(msg.value);

    } else {
        auto i = QImage(100, 20, QImage::Format_ARGB32);
        QPainter painter(&i);
        painter.fillRect(i.rect(), Qt::SolidPattern);
        painter.setPen(QPen(Qt::white));
        painter.drawText(i.rect(), "Cannot render image");
        return i;
    }

}
