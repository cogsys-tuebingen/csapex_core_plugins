/// HEADER
#include "output_display_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>
#include <QBoxLayout>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(OutputDisplayAdapter, csapex::OutputDisplay)


OutputDisplayAdapter::OutputDisplayAdapter(NodeHandleWeakPtr worker, std::weak_ptr<OutputDisplay> node, WidgetController* widget_ctrl)
    : DefaultNodeAdapter(worker, widget_ctrl), wrapped_(node), pixmap_(nullptr), view_(new QGraphicsView), empty(32, 32, QImage::Format_RGB16), painter(&empty), down_(false)
{
    auto n = wrapped_.lock();

    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));

    // translate to UI thread via Qt signal
    trackConnection(n->display_request.connect(std::bind(&OutputDisplayAdapter::displayRequest, this, std::placeholders::_1)));

    n->setAdapted();
}

OutputDisplayAdapter::~OutputDisplayAdapter()
{

}

bool OutputDisplayAdapter::eventFilter(QObject *o, QEvent *e)
{
    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::GraphicsSceneMousePress:
        down_ = true;
        last_pos_ = me->screenPos();
        e->accept();
        return true;
    case QEvent::GraphicsSceneMouseRelease:
        down_ = false;
        e->accept();
        return true;
    case QEvent::GraphicsSceneMouseMove:
        if(down_) {
            QPoint delta = me->screenPos() - last_pos_;

            last_pos_ = me->screenPos();

            state.width = std::max(32, view_->width() + delta.x());
            state.height = std::max(32, view_->height() + delta.y());

            view_->setFixedSize(QSize(state.width, state.height));
        }
        e->accept();
        return true;

    default:
        break;
    }

    return false;
}

void OutputDisplayAdapter::setupUi(QBoxLayout* layout)
{
    view_->setFixedSize(QSize(state.width, state.height));
    view_->setAcceptDrops(false);
    QGraphicsScene* scene = view_->scene();
    if(scene == nullptr) {
        scene = new QGraphicsScene();
        view_->setScene(scene);
        scene->installEventFilter(this);
    }

    layout->addWidget(view_);

    QHBoxLayout* sub = new QHBoxLayout;

    QPushButton* fit = new QPushButton("resize to content");
    sub->addWidget(fit, 0,  Qt::AlignLeft);
    QObject::connect(fit, SIGNAL(clicked()), this, SLOT(fitInView()));

    sub->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum));

    layout->addLayout(sub);

    connect(this, SIGNAL(displayRequest(QImage)), this, SLOT(display(QImage)));

    DefaultNodeAdapter::setupUi(layout);
}

void OutputDisplayAdapter::fitInView()
{
    if(last_size_.isNull()) {
        return;
    }

    state.width = last_size_.width();
    state.height = last_size_.height();
    view_->setFixedSize(QSize(state.width, state.height));
}

Memento::Ptr OutputDisplayAdapter::getState() const
{
    return std::shared_ptr<State>(new State(state));
}

void OutputDisplayAdapter::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<State> m = std::dynamic_pointer_cast<State> (memento);
    apex_assert(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
}

void OutputDisplayAdapter::display(const QImage& img)
{
    if(pixmap_ == nullptr) {
        if(view_->scene()) {
            delete view_->scene();
        }
        view_->setScene(new QGraphicsScene());
        view_->scene()->installEventFilter(this);

        pixmap_ = view_->scene()->addPixmap(QPixmap::fromImage(img));

    } else {
        pixmap_->setPixmap(QPixmap::fromImage(img));
    }

    last_size_ = img.size();

    view_->scene()->setSceneRect(img.rect());
    view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
    view_->scene()->update();
}

/// MOC
#include "moc_output_display_adapter.cpp"

