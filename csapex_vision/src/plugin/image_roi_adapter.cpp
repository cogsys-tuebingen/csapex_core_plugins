/// HEADER
#include "image_roi_adapter.h"

/// PROJECT
#include <csapex/model/connector_in.h>
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

CSAPEX_REGISTER_NODE_ADAPTER(ImageRoiAdapter, csapex::ImageRoi)


ImageRoiAdapter::ImageRoiAdapter(ImageRoi *node, WidgetController* widget_ctrl)
    : DefaultNodeAdapter(node, widget_ctrl),
      wrapped_(node),
      pixmap_(NULL),
      view_(new QGraphicsView),
      empty(32, 32, QImage::Format_RGB16),
      painter(&empty),
      middle_button_down_(false),
      left_button_down_(false)
{
    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));

    // translate to UI thread via Qt signal
    node->display_request.connect(boost::bind(&ImageRoiAdapter::displayRequest, this, _1));
    node->submit_request.connect(boost::bind(&ImageRoiAdapter::submitRequest, this));
}

bool ImageRoiAdapter::eventFilter(QObject *o, QEvent *e)
{
    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::KeyPress: {
        QKeyEvent* ke = dynamic_cast<QKeyEvent*>(e);

        int key = ke->key();
        break;
    }
    case QEvent::GraphicsSceneMousePress:
        if(me->button() == Qt::MiddleButton) {
            middle_button_down_ = true;
            middle_last_pos_ = me->screenPos();
            e->accept();
            return true;
        }
//        if(me->button() == Qt::LeftButton) {
//            left_button_down_ = true;
//            left_last_pos_ = me->screenPos();
//            e->accept();
//        }
        break;
    case QEvent::GraphicsSceneMouseRelease: {
        if(me->button() == Qt::MiddleButton) {
            middle_button_down_ = false;
            e->accept();
            return true;
        }
        if(me->button() == Qt::LeftButton) {
            submit();
            e->accept();
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
        break;
//        if(left_button_down_) {
//            QPoint delta   = me->screenPos() - left_last_pos_;
//            left_last_pos_ = me->screenPos();

//            QRectF r       = rect_->boundingRect();
//            QRectF scene_r = view_->sceneRect();
//            if(r.left() + delta.x() >= scene_r.left() &&
//                    r.top() + delta.y() >= scene_r.top() &&
//                    r.right() + delta.x() <= scene_r.right() &&
//                    r.bottom() + delta.y() <= scene_r.bottom()) {
//                std::cout << delta.x() << " " << delta.y() << std::endl;
//                rect_->moveBy(delta.x(), delta.y());
//            }

//        }
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

void ImageRoiAdapter::setupUi(QBoxLayout* layout)
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
    view_->scene()->installEventFilter(this);

    layout->addWidget(view_);

    QHBoxLayout* sub = new QHBoxLayout;
    QPushButton* fit = new QPushButton("fit size");
    sub->addWidget(fit, 0,  Qt::AlignLeft);
    QObject::connect(fit, SIGNAL(clicked()), this, SLOT(fitInView()));

    sub->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum));
    layout_->addLayout(sub);

    rect_ = new QGraphicsRectItem(0, 0, 0, 0);
    rect_->setPen(QPen(Qt::red));
    rect_->setFlags(QGraphicsItem::ItemIsMovable);
    view_->scene()->addItem(rect_);

    connect(this, SIGNAL(displayRequest(QSharedPointer<QImage>)), this, SLOT(display(QSharedPointer<QImage>)));
    connect(this, SIGNAL(submitRequest()), this, SLOT(submit()));

    DefaultNodeAdapter::setupUi(layout);
}

Memento::Ptr ImageRoiAdapter::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ImageRoiAdapter::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    apex_assert_hard(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
}

void ImageRoiAdapter::display(QSharedPointer<QImage> img)
{
    QPixmap pixmap = QPixmap::fromImage(*img);
    if(last_size_.width() != img->width() ||
            last_size_.height() != img->height()) {
        view_->scene()->setSceneRect(img->rect());
        view_->fitInView(view_->sceneRect(), Qt::KeepAspectRatio);
        result_.setX(0);
        result_.setY(0);
    }

    result_.setWidth(wrapped_->param<int>("roi width"));
    result_.setHeight(wrapped_->param<int>("roi height"));

    if(pixmap_ != NULL)
        view_->scene()->removeItem(pixmap_);

    pixmap_ = view_->scene()->addPixmap(pixmap);
    rect_->setRect(result_);
    rect_->setZValue(pixmap_->zValue() + 0.1);

    view_->scene()->update();

    last_size_ = img->size();
}

void ImageRoiAdapter::fitInView()
{
    if(last_size_.isNull()) {
        return;
    }
    state.width  = last_size_.width();
    state.height = last_size_.height();
    view_->setFixedSize(QSize(state.width, state.height));
    view_->fitInView(view_->sceneRect(), Qt::KeepAspectRatio);
}

void ImageRoiAdapter::submit()
{
    if(pixmap_ == NULL)
        return;

    connection_types::RoiMessage::Ptr result_msg(new connection_types::RoiMessage);

    QPointF top_left     = rect_->rect().topLeft() - view_->sceneRect().topLeft();
    QPointF bottom_right = rect_->rect().bottomRight() - view_->sceneRect().bottomRight();
    top_left.setX(std::min(pixmap_->pixmap().width(), std::max(0, (int) top_left.x())));
    top_left.setY(std::min(pixmap_->pixmap().height(), std::max(0, (int) top_left.y())));
    bottom_right.setX(std::min(pixmap_->pixmap().width(), std::max(0, (int) bottom_right.x())));
    bottom_right.setY(std::min(pixmap_->pixmap().height(), std::max(0, (int) bottom_right.y())));

    Roi roi(top_left.x(), top_left.y(),
            bottom_right.x() - top_left.x(),
            bottom_right.y() - top_left.y());

    if(roi.w() > 0 && roi.h() > 0)
        wrapped_->setResult(result_msg);

}
