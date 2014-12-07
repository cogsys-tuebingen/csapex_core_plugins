#ifndef TEXT_DISPLAY_ADAPTER_H
#define TEXT_DISPLAY_ADAPTER_H


/// PROJECT
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include "text_display.h"

/// SYSTEM
#include <QLabel>

namespace csapex {

class TextDisplayAdapter : public QObject, public NodeAdapter
{
    Q_OBJECT

public:
    TextDisplayAdapter(NodeWorker* worker, TextDisplay *node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(const std::string& txt);

Q_SIGNALS:
    void displayRequest(const std::string& txt);

protected:
    TextDisplay* wrapped_;

private:
    QLabel* txt_;
};

}
#endif // TEXT_DISPLAY_ADAPTER_H
