#ifndef TEXT_DISPLAY_ADAPTER_H
#define TEXT_DISPLAY_ADAPTER_H


/// PROJECT
#include <csapex/view/node/resizable_node_adapter.h>

/// COMPONENT
#include "text_display.h"

/// SYSTEM
#include <QLabel>

namespace csapex {

class TextDisplayAdapter : public QObject, public ResizableNodeAdapter
{
    Q_OBJECT

public:
    TextDisplayAdapter(NodeFacadePtr worker, NodeBox* parent);

    bool eventFilter(QObject* o, QEvent* e);

    virtual void setupUi(QBoxLayout* layout);
    virtual void resize(const QSize& size) override;

    virtual void setManualResize(bool manual) override;

public Q_SLOTS:
    void display(const std::string& txt);

Q_SIGNALS:
    void displayRequest(const std::string& txt);

private:
    QLabel* txt_;
};

}
#endif // TEXT_DISPLAY_ADAPTER_H
