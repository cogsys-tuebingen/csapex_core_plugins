#ifndef TEXT_DISPLAY_ADAPTER_H
#define TEXT_DISPLAY_ADAPTER_H


/// PROJECT
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include "text_display.h"

/// SYSTEM
#include <QTextEdit>

namespace csapex {

class TextDisplayAdapter : public QObject, public NodeAdapter
{
    Q_OBJECT

public:
    TextDisplayAdapter(TextDisplay *node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(const std::string& txt);

Q_SIGNALS:
    void displayRequest(const std::string& txt);

protected:
    TextDisplay* wrapped_;

private:
    QTextEdit* txt_;
};

}
#endif // TEXT_DISPLAY_ADAPTER_H
