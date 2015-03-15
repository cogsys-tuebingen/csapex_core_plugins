/// HEADER
#include <csapex_vision/image_renderer.h>

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <utils_qt/QtCvImageConverter.h>

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

QSharedPointer<QImage> ImageRenderer::doRender(const connection_types::CvMatMessage &msg)
{
    Encoding encoding = msg.getEncoding();

    if(encoding.matches(enc::rgb)) {
        cv::Mat mat = msg.value;
        return QSharedPointer<QImage>(new QImage((uchar*) mat.data,
                                                 mat.cols, mat.rows, mat.step,
                                                 QImage::Format_RGB888,
                                                 clean_mat, new mat_lifetime_extender(mat)));

    } else if(encoding.matches(enc::bgr) || encoding.matches(enc::mono)) {
        return QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(msg.value);

    } else if(encoding.matches(enc::depth)) {
        return QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(msg.value);

    } else {
        auto i = QSharedPointer<QImage>(new QImage(70, 20, QImage::Format_ARGB32));
        QPainter painter(i.data());
        painter.fillRect(i->rect(), Qt::SolidPattern);
        painter.setPen(QPen(Qt::white));
        painter.drawText(i->rect(), "No Image");
        return i;
    }

}
