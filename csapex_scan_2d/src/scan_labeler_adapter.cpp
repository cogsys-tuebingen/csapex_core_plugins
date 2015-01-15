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
#include <QScrollBar>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(ScanLabelerAdapter, csapex::ScanLabeler)


ScanLabelerAdapter::ScanLabelerAdapter(NodeWorker* worker, ScanLabeler *node, WidgetController* widget_ctrl)
    : DefaultNodeAdapter(worker, widget_ctrl), wrapped_(node), pixmap_(nullptr), view_(new QGraphicsView),
      resize_down_(false), move_down_(false)
{
    // translate to UI thread via Qt signal
    node->display_request.connect(std::bind(&ScanLabelerAdapter::displayRequest, this, std::placeholders::_1));
    node->submit_request.connect(std::bind(&ScanLabelerAdapter::submitRequest, this));
}

void ScanLabelerAdapter::labelSelected()
{
    int label = wrapped_->readParameter<int>("label");

    labelSelected(label);
    view_->scene()->clearSelection();
}

void ScanLabelerAdapter::labelSelected(int label)
{
    QBrush brush(color::fromCount(label), Qt::SolidPattern);
    QPen pen(color::fromCount(label));

    Q_FOREACH(QGraphicsItem* item, view_->scene()->selectedItems()) {
        result_->value.labels[item->data(0).toUInt()] = label;

        QGraphicsRectItem* rect = dynamic_cast<QGraphicsRectItem*> (item);
        if(rect) {
            rect->setPen(pen);
            rect->setBrush(brush);
        }
    }
    view_->update();
}

void ScanLabelerAdapter::updateLabel(int label)
{
    node_->getNode()->getParameter("label")->set(label);
}

bool ScanLabelerAdapter::eventFilter(QObject *o, QEvent *e)
{
    if(view_->signalsBlocked()) {
        return false;
    }

    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::KeyPress: {
        QKeyEvent* ke = dynamic_cast<QKeyEvent*>(e);

        int key = ke->key();

        if(Qt::Key_0 <= key && key <= Qt::Key_9) {
            updateLabel(ke->key() - Qt::Key_0);
        } else if(key == Qt::Key_Space) {
            submit();
        } else if(key == Qt::Key_Escape) {
            updateLabel(0);
        }

        break;
    }
    case QEvent::GraphicsSceneMousePress:
        if(me->button() == Qt::MiddleButton || me->button() == Qt::RightButton) {
            resize_down_ = me->button() == Qt::MiddleButton;
            move_down_ = me->button() == Qt::RightButton;
            last_pos_ = me->screenPos();
            e->accept();
            return true;
        }
        break;
    case QEvent::GraphicsSceneMouseRelease: {
        if(me->button() == Qt::MiddleButton || me->button() == Qt::RightButton) {
            resize_down_ = false;
            move_down_ = false;
            e->accept();
        }

        labelSelected();
        return true;
    }
    case QEvent::GraphicsSceneMouseMove:
        if(resize_down_ || move_down_) {
            QPoint delta = me->screenPos() - me->lastScreenPos();

            last_pos_ = me->screenPos();

            if(resize_down_) {
                state.width = std::max(32, view_->width() + delta.x());
                state.height = std::max(32, view_->height() + delta.y());
                view_->setFixedSize(QSize(state.width, state.height));

            } else if(move_down_) {
                double f = 1.0;
                view_->horizontalScrollBar()->setValue(view_->horizontalScrollBar()->value() + delta.x() * f);
                view_->verticalScrollBar()->setValue(view_->verticalScrollBar()->value() + delta.y() * f);
//                view_->translate(state.offset_x, state.offset_y);
            }


            e->accept();
            return true;
        }
        break;

    case QEvent::GraphicsSceneWheel: {
        e->accept();

        QGraphicsSceneWheelEvent* we = dynamic_cast<QGraphicsSceneWheelEvent*> (e);
        double scaleFactor = 1.1;

        view_->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
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
    if(scene == nullptr) {
        scene = new QGraphicsScene();
        view_->setScene(scene);
    }
    scene->installEventFilter(this);

    view_->setFixedSize(QSize(state.width, state.height));
    view_->setMouseTracking(true);
    view_->setAcceptDrops(false);
    view_->setDragMode(QGraphicsView::RubberBandDrag);
    view_->setContextMenuPolicy(Qt::PreventContextMenu);
    view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    view_->setInteractive(true);

    layout->addWidget(view_);

    connect(this, SIGNAL(displayRequest(const lib_laser_processing::Scan* )), this, SLOT(display(const lib_laser_processing::Scan* )));
    connect(this, SIGNAL(submitRequest()), this, SLOT(submit()));

    DefaultNodeAdapter::setupUi(layout);


    QRectF rect(-15.0 * SCALE, -15.0 * SCALE, 30.0 * SCALE, 30.0 * SCALE);
    scene->setSceneRect(rect);
    view_->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}

Memento::Ptr ScanLabelerAdapter::getState() const
{
    return std::shared_ptr<State>(new State(state));
}

void ScanLabelerAdapter::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<State> m = std::dynamic_pointer_cast<State> (memento);
    apex_assert_hard(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
}

void ScanLabelerAdapter::display(const lib_laser_processing::Scan *scan)
{
    result_.reset(new connection_types::LabeledScanMessage);

    QGraphicsScene* scene = view_->scene();

    scene->clear();


    float dim = SCALE * 0.05f;

    result_->value.rays = scan->rays;
    result_->value.labels.resize(scan->rays.size(), 0);

    QBrush brush(color::fromCount(0), Qt::SolidPattern);
    for(std::size_t i = 0, n =  scan->rays.size(); i < n; ++i) {
        const lib_laser_processing::LaserBeam& beam = scan->rays[i];
        QGraphicsItem* item = scene->addRect(SCALE * beam.pos(0), SCALE * beam.pos(1), dim, dim, QPen(brush.color()), brush);

        item->setData(0, QVariant::fromValue(i));

        item->setFlag(QGraphicsItem::ItemIsSelectable);
        item->setFlag(QGraphicsItem::ItemIsFocusable);
    }


    scene->update();
}

void ScanLabelerAdapter::submit()
{
    wrapped_->setResult(result_);
}
