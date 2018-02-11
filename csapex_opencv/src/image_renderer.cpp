/// HEADER
#include <csapex_opencv/image_renderer.h>

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/QtCvImageConverter.h>

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

std::unique_ptr<QImage> ImageRenderer::doRender(const connection_types::CvMatMessage &msg)
{
    Encoding encoding = msg.getEncoding();

    QImage result;

    if(!msg.value.empty()) {
        if(encoding.matches(enc::rgb)) {
            cv::Mat mat = msg.value;
            result = QImage((uchar*) mat.data,
                          mat.cols, mat.rows, mat.step,
                          QImage::Format_RGB888,
                          clean_mat, new mat_lifetime_extender(mat));

        } else if(encoding.matches(enc::bgr) || encoding.matches(enc::mono)) {
            result = QtCvImageConverter::Converter::mat2QImage(msg.value);

        } else if(encoding.matches(enc::depth)) {
            result = QtCvImageConverter::Converter::mat2QImage(msg.value);

        } else if(msg.value.channels() == 1 || msg.value.channels() == 3) {
            result = QtCvImageConverter::Converter::mat2QImage(msg.value);

        } else {
            result = QImage(100, 20, QImage::Format_ARGB32);
            QPainter painter(&result);
            painter.fillRect(result.rect(), Qt::SolidPattern);
            painter.setPen(QPen(Qt::white));
            painter.drawText(result.rect(), "Cannot render image");
        }
    }

    return std::unique_ptr<QImage>(new QImage(std::move(result)));
}
