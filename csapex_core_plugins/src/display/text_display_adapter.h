#ifndef TEXT_DISPLAY_ADAPTER_H
#define TEXT_DISPLAY_ADAPTER_H

/// PROJECT
#include <csapex/view/node/resizable_node_adapter.h>

/// COMPONENT
#include "text_display.h"

/// SYSTEM
#include <QLabel>

namespace csapex
{
class TextDisplayAdapter : public QObject, public ResizableNodeAdapter
{
    Q_OBJECT

public:
    TextDisplayAdapter(NodeFacadePtr worker, NodeBox* parent);
    ~TextDisplayAdapter();

    bool eventFilter(QObject* o, QEvent* e) override;

    void setupUi(QBoxLayout* layout) override;
    void resize(const QSize& size) override;

    void setManualResize(bool manual) override;

public Q_SLOTS:
    void display(const std::string& txt);

Q_SIGNALS:
    void displayRequest(const std::string& txt);

private:
    QLabel* txt_;
};

}  // namespace csapex
#endif  // TEXT_DISPLAY_ADAPTER_H
