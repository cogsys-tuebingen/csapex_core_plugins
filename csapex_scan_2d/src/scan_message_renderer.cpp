/// HEADER
#include "scan_message_renderer.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/QtCvImageConverter.h>

CSAPEX_REGISTER_CLASS(csapex::ScanMessageRenderer, csapex::MessageRenderer)
CSAPEX_REGISTER_CLASS(csapex::LabeledScanMessageRenderer, csapex::MessageRenderer)

using namespace csapex;

std::unique_ptr<QImage> ScanMessageRenderer::doRender(const connection_types::ScanMessage& msg)
{
    cv::Mat mat;
    renderer.render(msg.value, mat);
    return std::unique_ptr<QImage>(new QImage(std::move(QtCvImageConverter::Converter::mat2QImage(mat))));
}

std::vector<csapex::param::ParameterPtr> ScanMessageRenderer::getParameters() const
{
    return renderer.getParameters();
}

std::unique_ptr<QImage> LabeledScanMessageRenderer::doRender(const connection_types::LabeledScanMessage& msg)
{
    cv::Mat mat;
    renderer.render(msg.value, mat);
    return std::unique_ptr<QImage>(new QImage(std::move(QtCvImageConverter::Converter::mat2QImage(mat))));
}

std::vector<csapex::param::ParameterPtr> LabeledScanMessageRenderer::getParameters() const
{
    return renderer.getParameters();
}
