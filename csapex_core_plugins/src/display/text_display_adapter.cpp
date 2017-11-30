/// HEADER
#include "text_display_adapter.h"

/// PROJECT
#include <csapex/model/node_facade_local.h>
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>
#include <csapex/model/node_facade.h>
#include <csapex/io/raw_message.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <QBoxLayout>
#include <QEvent>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(TextDisplayAdapter, csapex::TextDisplay)


TextDisplayAdapter::TextDisplayAdapter(NodeFacadePtr node, NodeBox* parent)
    : ResizableNodeAdapter(node, parent)
{
    observe(node->raw_data_connection, [this](StreamableConstPtr msg) {
        if(std::shared_ptr<RawMessage const> raw = std::dynamic_pointer_cast<RawMessage const>(msg)) {
            SerializationBuffer buffer(raw->getData());
            std::string text;
            buffer >> text;
            displayRequest(text);
        }
    });;
}

namespace {
static bool isFixedPitch(const QFont & font) {
    const QFontInfo fi(font);
    return fi.fixedPitch();
}

static QFont getMonospaceFont(){
    QFont font("monospace");
    if (isFixedPitch(font)) return font;
    font.setStyleHint(QFont::Monospace);
    if (isFixedPitch(font)) return font;
    font.setStyleHint(QFont::TypeWriter);
    if (isFixedPitch(font)) return font;
    font.setFamily("courier");
    if (isFixedPitch(font)) return font;
    return font;
}
}

bool TextDisplayAdapter::eventFilter(QObject *o, QEvent *e)
{
    if (e->type() == QEvent::Resize){
        QSize s = txt_->sizeHint();
        setSize(s.width(), s.height());
    }

    return false;
}


void TextDisplayAdapter::setupUi(QBoxLayout* layout)
{
    txt_ = new QLabel;

    txt_->setFont(getMonospaceFont());
    layout->addWidget(txt_);
    txt_->installEventFilter(this);

    connect(this, &TextDisplayAdapter::displayRequest, this, &TextDisplayAdapter::display);

    ResizableNodeAdapter::setupUi(layout);
}

void TextDisplayAdapter::resize(const QSize& size)
{
    txt_->setFixedSize(size);
}

void TextDisplayAdapter::setManualResize(bool manual)
{
    if(manual) {
        txt_->setMinimumSize(QSize(10, 10));
    } else {
        txt_->setMinimumSize(txt_->size());
    }
}

void TextDisplayAdapter::display(const std::string& txt)
{
    txt_->setMaximumWidth(txt_->parentWidget()->width());
    txt_->setText(QString::fromStdString(txt));
}

/// MOC
#include "moc_text_display_adapter.cpp"

