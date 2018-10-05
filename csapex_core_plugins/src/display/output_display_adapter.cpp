/// HEADER
#include "output_display_adapter.h"

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node_facade_proxy.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/io.h>
#include <csapex/utility/assert.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QEvent>
#include <QPushButton>

using namespace csapex;

CSAPEX_REGISTER_LOCAL_NODE_ADAPTER(OutputDisplayDirectAdapter, csapex::OutputDisplay)

OutputDisplayDirectAdapter::OutputDisplayDirectAdapter(NodeFacadeImplementationPtr node, NodeBox* parent, std::weak_ptr<OutputDisplay> instance) : ResizableNodeAdapter(node, parent)
{
    auto n = instance.lock();
    // translate to UI thread via Qt signal
    observe(n->display_request, this, &OutputDisplayDirectAdapter::displayRequest);
}

OutputDisplayDirectAdapter::~OutputDisplayDirectAdapter()
{
}

bool OutputDisplayDirectAdapter::eventFilter(QObject* o, QEvent* e)
{
    if (e->type() == QEvent::Resize) {
        QSize s = label_view_->sizeHint();
        setSize(s.width(), s.height());
    }

    return false;
}

void OutputDisplayDirectAdapter::resize(const QSize& size)
{
    label_view_->setSize(size);
}

void OutputDisplayDirectAdapter::setupUi(QBoxLayout* layout)
{
    label_view_ = new ImageWidget;
    label_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    label_view_->installEventFilter(this);

    layout->addWidget(label_view_);

    QHBoxLayout* sub = new QHBoxLayout;

    QPushButton* fit = new QPushButton("resize to content");
    sub->addWidget(fit, 0, Qt::AlignLeft);
    QObject::connect(fit, SIGNAL(clicked()), this, SLOT(fitInView()));

    sub->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum));

    layout->addLayout(sub);

    connect(this, &OutputDisplayDirectAdapter::displayRequest, this, &OutputDisplayDirectAdapter::display);

    ResizableNodeAdapter::setupUi(layout);
}

void OutputDisplayDirectAdapter::setManualResize(bool manual)
{
    label_view_->setManualResize(manual);
}

void OutputDisplayDirectAdapter::fitInView()
{
    setSize(last_image_size_.width(), last_image_size_.height());

    doResize();
}

void OutputDisplayDirectAdapter::display(const QImage& img)
{
    last_image_size_ = img.size();
    label_view_->setPixmap(QPixmap::fromImage(img));
    label_view_->repaint();
}
