/// HEADER
#include "text_display_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_node_adapter.h>

/// SYSTEM
#include <QBoxLayout>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(TextDisplayAdapter, csapex::TextDisplay)


TextDisplayAdapter::TextDisplayAdapter(NodeWorkerWeakPtr worker, std::weak_ptr<TextDisplay> node, WidgetController* widget_ctrl)
    : NodeAdapter(worker, widget_ctrl), wrapped_(node)
{
    auto n = wrapped_.lock();

    // translate to UI thread via Qt signal
    trackConnection(n->display_request.connect(std::bind(&TextDisplayAdapter::displayRequest, this, std::placeholders::_1)));
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

void TextDisplayAdapter::setupUi(QBoxLayout* layout)
{
    txt_ = new QLabel;

    txt_->setFont(getMonospaceFont());
    layout->addWidget(txt_);

    connect(this, SIGNAL(displayRequest(std::string)), this, SLOT(display(std::string)));
}

void TextDisplayAdapter::display(const std::string& txt)
{
    txt_->setMaximumWidth(txt_->parentWidget()->width());
    txt_->setText(QString::fromStdString(txt));
}
/// MOC
#include "moc_text_display_adapter.cpp"

