#ifndef SCANMESSAGERENDERER_H
#define SCANMESSAGERENDERER_H

/// PROJECT
#include <csapex/msg/message_renderer.h>

/// COMPONENT
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex_scan_2d/renderer.h>
#include <csapex_scan_2d/scan_message.h>

namespace csapex
{
class ScanMessageRenderer : public MessageRendererImplementation<connection_types::ScanMessage>
{
public:
    virtual std::unique_ptr<QImage> doRender(const connection_types::ScanMessage& msg);
    virtual std::vector<csapex::param::ParameterPtr> getParameters() const;

private:
    Renderer renderer;
};

class LabeledScanMessageRenderer : public MessageRendererImplementation<connection_types::LabeledScanMessage>
{
public:
    virtual std::unique_ptr<QImage> doRender(const connection_types::LabeledScanMessage& msg);
    virtual std::vector<csapex::param::ParameterPtr> getParameters() const;

private:
    Renderer renderer;
};
}  // namespace csapex

#endif  // SCANMESSAGERENDERER_H
