/// HEADER
#include "scan_labeler_adapter.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/register_node_adapter.h>
#include <utils_qt/QtCvImageConverter.h>
#include <csapex/utility/color.hpp>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>
#include <QKeyEvent>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(ScanLabelerAdapter, csapex::ScanLabeler)


ScanLabelerAdapter::ScanLabelerAdapter(NodeWorker* worker, ScanLabeler *node, WidgetController* widget_ctrl)
    : DefaultNodeAdapter(worker, widget_ctrl), wrapped_(node), pixmap_(NULL), view_(new QGraphicsView), empty(32, 32, QImage::Format_RGB16), painter(&empty), label_(0), down_(false)
{
    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));


    // translate to UI thread via Qt signal
    node->display_request.connect(boost::bind(&ScanLabelerAdapter::displayRequest, this, _1));
    node->submit_request.connect(boost::bind(&ScanLabelerAdapter::submitRequest, this));
}

void ScanLabelerAdapter::labelSelected()
{
    labelSelected(label_);
    view_->scene()->clearSelection();
}

void ScanLabelerAdapter::labelSelected(int label)
{
    QBrush brush(color::fromCount(label), Qt::SolidPattern);
    QPen pen(color::fromCount(label));

    Q_FOREACH(QGraphicsItem* item, view_->scene()->selectedItems()) {
        result_->value.labels[item->data(0).toUInt()] = label;

        QGraphicsEllipseItem* ellipse = dynamic_cast<QGraphicsEllipseItem*> (item);
        if(ellipse) {
            ellipse->setPen(pen);
            ellipse->setBrush(brush);
        }
    }
    view_->update();
}

void ScanLabelerAdapter::setLabel(int label)
{
    label_ = label;
    node_->getNode()->getParameter("label")->set(label_);
}

bool ScanLabelerAdapter::eventFilter(QObject *o, QEvent *e)
{
    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::KeyPress: {
        QKeyEvent* ke = dynamic_cast<QKeyEvent*>(e);

        int key = ke->key();

        if(Qt::Key_0 <= key && key <= Qt::Key_9) {
            setLabel(ke->key() - Qt::Key_0);
        } else if(key == Qt::Key_Space) {
            submit();
        } else if(key == Qt::Key_Escape) {
            setLabel(0);
        }

        break;
    }
    case QEvent::GraphicsSceneMousePress:
        if(me->button() == Qt::MiddleButton) {
            down_ = true;
            last_pos_ = me->screenPos();
            e->accept();
        }
        return true;
    case QEvent::GraphicsSceneMouseRelease: {
        if(me->button() == Qt::MiddleButton) {
            down_ = false;
            e->accept();
        }

        labelSelected();
        return true;
    }
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
    case QEvent::GraphicsSceneWheel: {
        e->accept();

        QGraphicsSceneWheelEvent* we = dynamic_cast<QGraphicsSceneWheelEvent*> (e);
        double scaleFactor = 1.1;
        if(we->delta() > 0) {
            // Zoom in
            view_->scale(scaleFactor, scaleFactor);
        } else {
            // Zooming out
            view_->scale(1.0 / scaleFactor, 1.0 / scaleFactor);
        }
        return true;
    }
    default:
        break;
    }
    return false;
}

void ScanLabelerAdapter::setupUi(QBoxLayout* layout)
{
    QGraphicsScene* scene = view_->scene();
    if(scene == NULL) {
        scene = new QGraphicsScene();
        view_->setScene(scene);
        scene->installEventFilter(this);
    }

    view_->setFixedSize(QSize(state.width, state.height));
    view_->setMouseTracking(true);
    view_->setAcceptDrops(false);
    view_->setDragMode(QGraphicsView::RubberBandDrag);

    layout->addWidget(view_);

    connect(this, SIGNAL(displayRequest(lib_laser_processing::Scan* )), this, SLOT(display(lib_laser_processing::Scan* )));
    connect(this, SIGNAL(submitRequest()), this, SLOT(submit()));

    DefaultNodeAdapter::setupUi(layout);

    setLabel(wrapped_->readParameter<int>("label"));
}

Memento::Ptr ScanLabelerAdapter::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ScanLabelerAdapter::setParameterState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    apex_assert_hard(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
}

void ScanLabelerAdapter::display(lib_laser_processing::Scan* scan)
{
    result_.reset(new connection_types::LabeledScanMessage);

    QGraphicsScene* scene = view_->scene();

    scene->clear();

    QRectF rect(-5.0, -5.0, 10.0, 10.0);


    float dim = 0.05f;

    result_->value.rays = scan->rays;
    result_->value.labels.resize(scan->rays.size(), 0);

    QBrush brush(color::fromCount(0), Qt::SolidPattern);
    for(std::size_t i = 0, n = scan->rays.size(); i < n; ++i) {
        const lib_laser_processing::LaserBeam& beam = scan->rays[i];
        QGraphicsEllipseItem* item = scene->addEllipse(beam.pos(0), beam.pos(1), dim, dim, QPen(brush.color()), brush);

        item->setData(0, QVariant::fromValue(i));

        item->setFlag(QGraphicsItem::ItemIsSelectable);
        item->setFlag(QGraphicsItem::ItemIsFocusable);
    }


    scene->setSceneRect(rect);
    view_->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    scene->update();
}

void ScanLabelerAdapter::submit()
{
    wrapped_->setResult(result_);
}
