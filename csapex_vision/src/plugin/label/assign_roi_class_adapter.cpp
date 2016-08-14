/// HEADER
#include "assign_roi_class_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/utility/assert.h>
#include <cslibs_vision/utils/histogram.hpp>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>
#include <QKeyEvent>

using namespace csapex;
using namespace vision_plugins;

CSAPEX_REGISTER_NODE_ADAPTER_NS(vision_plugins, AssignROIClassAdapter, vision_plugins::AssignROIClass)

namespace vision_plugins {
    class QInteractiveRect : public QGraphicsRectItem {
    public:
        QInteractiveRect(csapex::Roi &roi, float tx, float ty,
                         QColor &active_color, int &active_class) :
            QGraphicsRectItem(roi.x() + tx, roi.y() + ty,
                              roi.w(), roi.h()),
            roi_(roi),
            active_color_(active_color),
            active_class_(active_class)
        {
            //                                    setFlag(QGraphicsItem::ItemIsSelectable);

            cv::Scalar color = roi.color();
            QBrush brush(QColor(color[2], color[1], color[0], 127));
            setPen(QPen(QColor(color[2], color[1], color[0], 255)));
            setBrush(brush);
        }

        void setColor(const QColor &q)
        {
            QBrush brush(QColor(q.red(),
                                q.green(),
                                q.blue(),
                                127));
            QPen pen(QColor(q.red(),
                            q.green(),
                            q.blue(),
                            255));
            setPen(pen);
            setBrush(brush);
        }

        void updateCheck(const QPointF &pos)
        {
            QRectF rect = sceneBoundingRect();
            if(rect.contains(pos)) {
                update();
            }
        }

    protected:
        csapex::Roi &roi_;
        QColor      &active_color_;
        int         &active_class_;



        void update()
        {
            roi_.setClassification(active_class_);
            roi_.setColor(cv::Scalar(active_color_.blue(),
                                     active_color_.green(),
                                     active_color_.red()));
            setColor(active_color_);
        }

//        void mousePressEvent  (QGraphicsSceneMouseEvent *event)
//        {
//            event->accept();
//            std::cout << "Press" << std::endl;
//            update();
//        }

//        void mouseMoveEvent   (QGraphicsSceneMouseEvent *event)
//        {
//            event->accept();
//            std::cout << "Move" << std::endl;
//            update();
//        }

//        void mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
//        {
//            event->accept();
//            std::cout << "Release" << std::endl;
//            update();
//        }
    };
}


AssignROIClassAdapter::AssignROIClassAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<AssignROIClass> node)
    : DefaultNodeAdapter(worker, parent),
      wrapped_(node),
      active_class_(0),
      pixmap_(nullptr),
      view_(new QGraphicsView),
      empty(32, 32, QImage::Format_RGB16),
      painter(&empty),
      middle_button_down_(false),
      left_button_down_(false),
      loaded_(false)
{
    auto n = wrapped_.lock();

    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));

    // translate to UI thread via Qt signal
    trackConnection(n->display_request.connect(std::bind(&AssignROIClassAdapter::displayRequest, this,
                                            std::placeholders::_1)));
    trackConnection(n->set_class.connect(std::bind(&AssignROIClassAdapter::setClassRequest, this, std::placeholders::_1)));
    trackConnection(n->set_color.connect(std::bind(&AssignROIClassAdapter::setColorRequest, this,
                                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)));
    trackConnection(n->submit_request.connect(std::bind(&AssignROIClassAdapter::submitRequest, this)));
    trackConnection(n->drop_request.connect(std::bind(&AssignROIClassAdapter::dropRequest, this)));
    trackConnection(n->clear_request.connect(std::bind(&AssignROIClassAdapter::clearRequest, this)));
}

bool AssignROIClassAdapter::eventFilter(QObject *o, QEvent *e)
{
    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::KeyPress: {
        break;
    }
    case QEvent::GraphicsSceneMousePress:
        if(me->button() == Qt::MiddleButton) {
            middle_button_down_ = true;
            middle_last_pos_ = me->screenPos();
            e->accept();
            return true;
        }
        if(me->button() == Qt::LeftButton) {
            left_button_down_ = true;
            e->accept();
            return true;
        }
        break;
    case QEvent::GraphicsSceneMouseRelease: {
        if(me->button() == Qt::MiddleButton) {
            middle_button_down_ = false;
            e->accept();
            return true;
        }
        if(me->button() == Qt::LeftButton) {
            left_button_down_ = false;
            e->accept();

            QPointF mpos = me->scenePos();
            for(QInteractiveRect *i : rectangles_) {
                i->updateCheck(mpos);
            }


            return true;
        }
    }
        break;
    case QEvent::GraphicsSceneMouseMove:
        if(middle_button_down_) {
            QPoint delta     = me->screenPos() - middle_last_pos_;

            middle_last_pos_ = me->screenPos();

            state.width = std::max(32, view_->width() + delta.x());
            state.height = std::max(32, view_->height() + delta.y());

            view_->setFixedSize(QSize(state.width, state.height));
            e->accept();
            return true;
        }
        if(left_button_down_) {
            e->accept();
            QPointF mpos = me->scenePos();
            for(QInteractiveRect *i : rectangles_) {
                i->updateCheck(mpos);
            }

            return true;
        }
        break;
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

    view_->scene()->update();

    return false;
}

void AssignROIClassAdapter::setupUi(QBoxLayout* layout)
{
    QGraphicsScene* scene = view_->scene();
    if(scene == nullptr) {
        scene = new QGraphicsScene();
        view_->setScene(scene);
        scene->installEventFilter(this);
    }

    view_->setFixedSize(QSize(state.width, state.height));
    view_->setMouseTracking(true);
    view_->setAcceptDrops(false);
    view_->setDragMode(QGraphicsView::RubberBandDrag);
    view_->scene()->installEventFilter(this);

    layout->addWidget(view_);

    QHBoxLayout* sub = new QHBoxLayout;
    QPushButton* fit = new QPushButton("fit size");
    sub->addWidget(fit, 0,  Qt::AlignLeft);
    QObject::connect(fit, SIGNAL(clicked()), this, SLOT(fitInView()));

    sub->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum));
    layout_->addLayout(sub);

    pixmap_ = new QGraphicsPixmapItem;
    view_->scene()->addItem(pixmap_);

    connect(this, SIGNAL(displayRequest(QImage)), this, SLOT(display(QImage)));
    connect(this, SIGNAL(submitRequest()), this, SLOT(submit()));
    connect(this, SIGNAL(dropRequest()), this, SLOT(drop()));
    connect(this, SIGNAL(clearRequest()), this, SLOT(clear()));
    connect(this, SIGNAL(setColorRequest(int,int,int)), this, SLOT(setColor(int,int,int)));
    connect(this, SIGNAL(setClassRequest(int)), this, SLOT(setClass(int)));

    DefaultNodeAdapter::setupUi(layout);
}

Memento::Ptr AssignROIClassAdapter::getState() const
{
    return std::shared_ptr<State>(new State(state));
}

void AssignROIClassAdapter::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<State> m = std::dynamic_pointer_cast<State> (memento);
    apex_assert_hard(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
    loaded_ = true;
}

void AssignROIClassAdapter::display(QImage img)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    /// PREPARE LABLES
    img_ = img;

    QPixmap pixmap = QPixmap::fromImage(img_);

    if(!rectangles_.empty()) {
        for(QGraphicsRectItem* r : rectangles_) {
            view_->scene()->removeItem(r);
            delete r;
        }
        rectangles_.clear();
    }

    for(auto it = node->rois_.begin() ;
        it != node->rois_.end() ;
        ++it) {
        csapex::Roi &roi = it->value;

        QPointF ppos = pixmap_->scenePos();
        QInteractiveRect *rect = new QInteractiveRect(roi,
                                                      -ppos.x(), -ppos.y(),
                                                      active_color_,
                                                      active_class_);

        cv::Scalar color = roi.color();
        QColor qcolor(color[2], color[1], color[0]);
        rect->setColor(qcolor);
//        rect->setFlag(QGraphicsItem::ItemIsSelectable);
        view_->scene()->addItem(rect);
        rectangles_.push_back(rect);
    }


    bool change = state.last_size != img_.size();
    if(change || loaded_) {
        view_->scene()->setSceneRect(img_.rect());
        view_->fitInView(view_->sceneRect(), Qt::KeepAspectRatio);
        loaded_ = false;
    }

    if(pixmap_ != nullptr)
        pixmap_->setPixmap(pixmap);

    view_->scene()->update();

    state.last_size = img_.size();
}

void AssignROIClassAdapter::fitInView()
{
    if(state.last_size.isNull()) {
        return;
    }
    state.width  = state.last_size.width();
    state.height = state.last_size.height();
    view_->setFixedSize(QSize(state.width, state.height));
    view_->fitInView(view_->sceneRect(), Qt::KeepAspectRatio);
}

void AssignROIClassAdapter::submit()
{
    if(pixmap_ == nullptr)
        return;
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    node->done();
}

void AssignROIClassAdapter::drop()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    for(auto it = node->rois_.begin() ;
        it != node->rois_.end() ;
        ++it) {
        it->value.setClassification(-1);
    }
    node->done();
}

void AssignROIClassAdapter::clear()
{

}

void AssignROIClassAdapter::setColor(int r, int g, int b)
{
    QColor c(r,g,b,127);
    colors_[active_class_] = c;
    active_color_ = c;
}

void AssignROIClassAdapter::setClass(int c)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    active_class_ = c;
    QColor &col = colors_[c];
    node->setActiveClassColor(col.red(),col.green(), col.blue());
    active_color_ = col;
}

/// MOC
#include "../../moc_assign_roi_class_adapter.cpp"
