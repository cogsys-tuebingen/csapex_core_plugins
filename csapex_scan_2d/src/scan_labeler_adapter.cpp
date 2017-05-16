/// HEADER
#include "scan_labeler_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/utility/assert.h>
#include <csapex/model/node_facade.h>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>
#include <QKeyEvent>
#include <QScrollBar>
#include <QApplication>
#include <QLayout>

using namespace csapex;

CSAPEX_REGISTER_LEGACY_NODE_ADAPTER(ScanLabelerAdapter, csapex::ScanLabeler)


ScanLabelerAdapter::ScanLabelerAdapter(NodeFacadeWeakPtr worker, NodeBox* parent, std::weak_ptr<ScanLabeler> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node), view_(new QGraphicsView),
      resize_down_(false), move_down_(false)
{
    auto n = wrapped_.lock();

    // translate to UI thread via Qt signal
    trackConnection(n->display_request.connect(std::bind(&ScanLabelerAdapter::displayRequest, this, std::placeholders::_1)));
    trackConnection(n->submit_request.connect(std::bind(&ScanLabelerAdapter::submitRequest, this)));
}

void ScanLabelerAdapter::labelSelected()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    int label = node->readParameter<int>("label");

    labelSelected(label);
    view_->scene()->clearSelection();
}

void ScanLabelerAdapter::labelSelected(int label)
{
    QBrush brush(color::fromCount<QColor>(label), Qt::SolidPattern);
    QPen pen(color::fromCount<QColor>(label));

    for(QGraphicsItem* item : view_->scene()->selectedItems()) {
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
    NodeFacadePtr node_facade = node_.lock();
    if(node_facade) {
        if(auto node = wrapped_.lock()) {
            node->getParameter("label")->set(label);
        }
    }
}

void ScanLabelerAdapter::updatePolygon()
{
    state.inside_item->setPolygon(state.inside);

    labelInside();
}

void ScanLabelerAdapter::labelInside()
{
    for(QGraphicsItem* item : view_->scene()->collidingItems(state.inside_item)) {
        item->setSelected(true);
    }
    labelSelected();
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
        if(QApplication::keyboardModifiers() & Qt::ShiftModifier) {
            if(me->button() == Qt::LeftButton) {
                state.inside.push_back(me->scenePos());
                updatePolygon();
                return true;
            } else if(me->button() == Qt::RightButton) {
                state.inside.clear();
                updatePolygon();
                return true;
            }
        } else {
            if(me->button() == Qt::MiddleButton || me->button() == Qt::RightButton) {
                resize_down_ = me->button() == Qt::MiddleButton;
                move_down_ = me->button() == Qt::RightButton;
                last_pos_ = me->screenPos();
                e->accept();
                return true;
            }
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

        } else {
            std::stringstream ss;
            double dx = me->scenePos().x();
            double dy = me->scenePos().y();
            ss << "distance: " <<  std::hypot(dx, dy) / SCALE;
            view_->setToolTip(QString::fromStdString(ss.str()));
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
    view_->setAcceptDrops(false);
    view_->setDragMode(QGraphicsView::RubberBandDrag);
    view_->setContextMenuPolicy(Qt::PreventContextMenu);
    view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    view_->setInteractive(true);

    layout->addWidget(view_);

    connect(this, SIGNAL(displayRequest(const lib_laser_processing::Scan* )), this, SLOT(display(const lib_laser_processing::Scan* )));
    connect(this, SIGNAL(submitRequest()), this, SLOT(submit()));

    DefaultNodeAdapter::setupUi(layout);


    QRectF rect(-10.0 * SCALE, -15.0 * SCALE, 30.0 * SCALE, 30.0 * SCALE);
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
    apex_assert(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
}

void ScanLabelerAdapter::display(const lib_laser_processing::Scan *scan)
{
    NodeFacadePtr node_facade = node_.lock();
    if(!node_facade) {
        return;
    }

    result_.reset(new connection_types::LabeledScanMessage);

    QGraphicsScene* scene = view_->scene();

    scene->clear();


    float dim = SCALE * 0.05f;

    lib_laser_processing::Scan& s = result_->value;
    s = *scan;

    result_->value.rays = scan->rays;
    result_->value.labels.resize(scan->rays.size(), 0);

    QBrush brush(color::fromCount<QColor>(0), Qt::SolidPattern);
    for(std::size_t i = 0, n =  scan->rays.size(); i < n; ++i) {
        const lib_laser_processing::LaserBeam& beam = scan->rays[i];
        QGraphicsItem* item = scene->addRect(SCALE * beam.posX(), SCALE * beam.posY(), dim, dim, QPen(brush.color()), brush);

        item->setData(0, QVariant::fromValue(i));

        item->setFlag(QGraphicsItem::ItemIsSelectable);
        item->setFlag(QGraphicsItem::ItemIsFocusable);
    }

    state.inside_item = scene->addPolygon(state.inside);
    labelInside();

    scene->update();

    if(auto node = wrapped_.lock()) {
        if(node->readParameter<bool>("automatic")) {
            submit();
        }
    }
}

void ScanLabelerAdapter::submit()
{
    if(auto node = wrapped_.lock()) {
        node->setResult(result_);
    }
}
/// MOC
#include "moc_scan_labeler_adapter.cpp"

