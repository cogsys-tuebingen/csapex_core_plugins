/// HEADER
#include "mask_refinement_adapter.h"

/// PROJECT
#include <csapex/model/node_facade_impl.h>
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>
#include <QBitmap>
#include <yaml-cpp/yaml.h>

using namespace csapex;

CSAPEX_REGISTER_LOCAL_NODE_ADAPTER(MaskRefinementAdapter, csapex::MaskRefinement)


MaskRefinementAdapter::MaskRefinementAdapter(NodeFacadeImplementationPtr worker, NodeBox* parent, std::weak_ptr<MaskRefinement> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node),
      view_(new QGraphicsView),
      refresh_(false),
      brush_size_(1)
{
    auto n = wrapped_.lock();

    // translate to UI thread via Qt signal
    observe(n->next_image, this, &MaskRefinementAdapter::nextRequest);
    observe(n->update_brush, this, &MaskRefinementAdapter::updateBrushRequest);
    observe(n->input, std::bind(&MaskRefinementAdapter::inputRequest, this, std::placeholders::_1, std::placeholders::_2));

    QObject::connect(this, SIGNAL(nextRequest()), this, SLOT(next()));
    QObject::connect(this, SIGNAL(inputRequest(QImage, QImage)), this, SLOT(setMask(QImage, QImage)));
    QObject::connect(this, SIGNAL(updateBrushRequest()), this, SLOT(updateBrush()));
}

MaskRefinementAdapter::~MaskRefinementAdapter()
{
}

Memento::Ptr MaskRefinementAdapter::getState() const
{
    return std::shared_ptr<State>(new State(state));
}

void MaskRefinementAdapter::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<State> m = std::dynamic_pointer_cast<State> (memento);
    apex_assert(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
}

void MaskRefinementAdapter::setupUi(QBoxLayout *layout)
{
    QGraphicsScene* scene = view_->scene();
    if(scene == nullptr) {
        scene = new QGraphicsScene();
        view_->setScene(scene);
        scene->installEventFilter(this);
        view_->installEventFilter(this);
    }
    mask_pixmap_ = new QGraphicsPixmapItem;
    masked_red_pixmap_ = new QGraphicsPixmapItem;
    masked_bw_pixmap_ = new QGraphicsPixmapItem;

    view_->scene()->addItem(mask_pixmap_);
    view_->scene()->addItem(masked_red_pixmap_);
    view_->scene()->addItem(masked_bw_pixmap_);

    view_->setContextMenuPolicy(Qt::PreventContextMenu);
    view_->setMouseTracking(true);

    cursor_ = new QGraphicsEllipseItem;
    cursor_->setPen(QPen(Qt::red, 1));
    view_->scene()->addItem(cursor_);

    layout->addWidget(view_);

    DefaultNodeAdapter::setupUi(layout);
}

bool MaskRefinementAdapter::eventFilter(QObject *o, QEvent *e)
{
    auto node = wrapped_.lock();
    if(!node) {
        return false;
    }

    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::KeyPress: {
        break;
    }

    case QEvent::Resize:
    {
        if(refresh_) {
            fitView();
            refresh_ = false;
        }
        return false;
    }

    case QEvent::GraphicsSceneMousePress:
        if(me->button() == Qt::MiddleButton) {
            middle_last_pos_ = me->screenPos();

        } else if(me->button() == Qt::LeftButton) {
            draw(me->scenePos(), Qt::white);

        } else if(me->button() == Qt::RightButton) {
            draw(me->scenePos(), Qt::black);
        }

        e->accept();
        return true;

    case QEvent::GraphicsSceneMouseRelease: {
        e->accept();
        return true;
    }

    case QEvent::GraphicsSceneMouseMove:
        if(me->buttons() & Qt::MiddleButton) {
            auto scene = view_->scene();
            if(!scene) {
                return false;
            }

            QPoint delta     = me->screenPos() - middle_last_pos_;

            middle_last_pos_ = me->screenPos();

            state.width = std::max(32, view_->width() + delta.x());
            state.height = std::max(32, view_->height() + delta.y());

            view_->setFixedSize(QSize(state.width, state.height));

            refresh_ = true;

            e->accept();

        } else if(me->buttons() & Qt::LeftButton) {
            draw(me->scenePos(), Qt::white);

        } else if(me->buttons() & Qt::RightButton) {
            draw(me->scenePos(), Qt::black);
        }


        updateCursor(me->scenePos());

        return true;

        break;
    case QEvent::GraphicsSceneWheel: {
        e->accept();

        QGraphicsSceneWheelEvent* we = dynamic_cast<QGraphicsSceneWheelEvent*> (e);

        int delta = we->delta() > 0 ? 1 : -1;
        brush_size_ = std::max(1, std::min(64, brush_size_ + delta));

        node->setParameter<int>("brush/size", brush_size_);

        updateCursor(we->scenePos());

        return true;
    }
    default:
        break;
    }
    return false;
}

void MaskRefinementAdapter::maskImage()
{
    if(masked_red_pixmap_->isVisible()) {
        masked_red_pm_ = QPixmap::fromImage(img_);

        QPainter img_painter_red(&masked_red_pm_);
        img_painter_red.setClipRegion(QRegion(QBitmap(mask_pm_)));
        img_painter_red.fillRect(masked_red_pm_.rect(), QBrush(QColor(255, 0, 0, 128)));

        masked_red_pixmap_->setPixmap(masked_red_pm_);
        masked_red_pixmap_->setPos(0, mask_pixmap_->boundingRect().height());

        masked_bw_pm_ = QPixmap::fromImage(img_);

        QPainter img_painter_black(&masked_bw_pm_);
        img_painter_black.setClipRegion(QRegion(QBitmap(mask_pm_)));
        img_painter_black.fillRect(masked_bw_pm_.rect(), QBrush(QColor(0, 0, 0, 255)));

        masked_bw_pixmap_->setPixmap(masked_bw_pm_);
        masked_bw_pixmap_->setPos(mask_pixmap_->boundingRect().width(), 0);
    }
}


void MaskRefinementAdapter::draw(const QPointF &pos, Qt::GlobalColor color)
{
    double x = pos.x();
    double y = pos.y();

    auto bound = mask_pixmap_->boundingRect();
    double w = bound.width();
    double h = bound.height();
    if(x >= w) {
        x -= w;
    }
    if(y >= h) {
        y -= h;
    }

    QPainter painter(&mask_);
    painter.setRenderHint(QPainter::Antialiasing, false);
    QPen pen(color, 1);
    painter.setPen(pen);
    QBrush brush(color);
    painter.setBrush(brush);

    int r = std::max(1, brush_size_ / 2);
    painter.drawEllipse(x-r/2,y-r/2, r, r);
    painter.end();

    mask_pm_ = QPixmap::fromImage(mask_);
    mask_pixmap_->setPixmap(mask_pm_);

    maskImage();

    mask_pixmap_->setZValue(0.0);
    cursor_->setZValue(1.0);

    view_->update();
}

void MaskRefinementAdapter::updateCursor(const QPointF& pos)
{
    double x = pos.x();
    double y = pos.y();

    if(x < 0) x = 0;
    if(y < 0) y = 0;

    double w = mask_pixmap_->boundingRect().width() + masked_bw_pixmap_->boundingRect().width();
    double h = mask_pixmap_->boundingRect().height() + masked_red_pixmap_->boundingRect().height();
    if(x >= w) x = w;
    if(y >= h) y = h;

    int r = std::max(1, brush_size_ / 2);
    cursor_->setRect(x-r/2, y-r/2,  r, r);
    cursor_->update();
    view_->update();
}

void MaskRefinementAdapter::setMask(QImage mask, QImage masked)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    updateBrush();

    mask_ = mask;
    mask_pm_ = QPixmap::fromImage(mask_);
    mask_pixmap_->setPixmap(mask_pm_);

    if(masked.isNull()) {
        masked_red_pixmap_->hide();
    } else {
        img_ = masked;
        masked_red_pixmap_->show();
        masked_bw_pixmap_->show();
        maskImage();
    }

    fitView();
}

void MaskRefinementAdapter::fitView()
{
    view_->setFixedSize(QSize(state.width, state.height));

    QRectF rect = mask_pixmap_->sceneBoundingRect();
    rect |= masked_red_pixmap_->sceneBoundingRect();
    rect |= masked_bw_pixmap_->sceneBoundingRect();
    view_->fitInView(rect, Qt::KeepAspectRatio);
    view_->update();
}


void MaskRefinementAdapter::next()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    node->setMask(mask_);
}

void MaskRefinementAdapter::updateBrush()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    brush_size_ = node->readParameter<int>("brush/size");
}

/// MOC
#include "moc_mask_refinement_adapter.cpp"
