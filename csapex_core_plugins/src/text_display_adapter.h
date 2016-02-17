#ifndef TEXT_DISPLAY_ADAPTER_H
#define TEXT_DISPLAY_ADAPTER_H


/// PROJECT
#include <csapex/view/node/node_adapter.h>

/// COMPONENT
#include "text_display.h"

/// SYSTEM
#include <QLabel>

namespace csapex {

class TextDisplayAdapter : public QObject, public NodeAdapter
{
    Q_OBJECT

public:
    TextDisplayAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<TextDisplay> node);

    virtual void setupUi(QBoxLayout* layout);

    virtual bool isResizable() const override;

public Q_SLOTS:
    void display(const std::string& txt);

Q_SIGNALS:
    void displayRequest(const std::string& txt);

protected:
    std::weak_ptr<TextDisplay> wrapped_;

private:
    QLabel* txt_;
};

}
#endif // TEXT_DISPLAY_ADAPTER_H
