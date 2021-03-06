#ifndef OUTPUT_DISPLAY_ADAPTER_PROXY_H
#define OUTPUT_DISPLAY_ADAPTER_PROXY_H

/// PROJECT
#include <csapex/view/node/resizable_node_adapter.h>
#include <csapex_core_plugins/image_widget.h>
#include <csapex/utility/yaml.h>

namespace csapex
{
class ImageWidget;

class OutputDisplayAdapter : public QObject, public ResizableNodeAdapter
{
    Q_OBJECT

public:
    OutputDisplayAdapter(NodeFacadePtr node, NodeBox* parent);
    ~OutputDisplayAdapter();

    void setupUi(QBoxLayout* layout) override;

    void setManualResize(bool manual) override;

public Q_SLOTS:
    void display(const QImage& img);
    void fitInView();

Q_SIGNALS:
    void displayRequest(QImage img);

protected:
    bool eventFilter(QObject* o, QEvent* e) override;
    void resize(const QSize& size) override;

private:
    QSize last_image_size_;

    ImageWidget* label_view_;
};

}  // namespace csapex

#endif  // OUTPUT_DISPLAY_ADAPTER_PROXY_H
