/// HEADER
#include "scan_message_renderer.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <utils_qt/QtCvImageConverter.h>

CSAPEX_REGISTER_CLASS(csapex::ScanMessageRenderer, csapex::MessageRenderer)
CSAPEX_REGISTER_CLASS(csapex::LabeledScanMessageRenderer, csapex::MessageRenderer)

using namespace csapex;


QSharedPointer<QImage> ScanMessageRenderer::doRender(const connection_types::ScanMessage &msg)
{
    cv::Mat mat;
    renderer.render(msg.value, mat);
    return QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mat);
}

QSharedPointer<QImage> LabeledScanMessageRenderer::doRender(const connection_types::LabeledScanMessage &msg)
{
    cv::Mat mat;
    renderer.render(msg.value, mat);
    return QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mat);
}
