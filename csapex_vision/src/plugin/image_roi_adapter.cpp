/// HEADER
#include "image_roi_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
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

ImageRoiAdapter::ImageRoiAdapter(NodeWorker* worker, ImageRoi *node, WidgetController* widget_ctrl)
    : DefaultNodeAdapter(worker, widget_ctrl),
      wrapped_(node),
      pixmap_(nullptr),
      view_(new QGraphicsView),
      empty(32, 32, QImage::Format_RGB16),
      painter(&empty),
      middle_button_down_(false),
      loaded_(false)
{
    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));

    // translate to UI thread via Qt signal
    node->display_request.connect(std::bind(&ImageRoiAdapter::displayRequest, this, std::placeholders::_1));
    node->submit_request.connect(std::bind(&ImageRoiAdapter::submitRequest, this));
    node->drop_request.connect(std::bind(&ImageRoiAdapter::dropRequest, this));
}

bool ImageRoiAdapter::eventFilter(QObject *o, QEvent *e)
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
        break;
    case QEvent::GraphicsSceneMouseRelease: {
        if(me->button() == Qt::MiddleButton) {
            middle_button_down_ = false;
            e->accept();
            return true;
        }
        if(me->button() == Qt::LeftButton) {
            if(!wrapped_->readParameter<bool>("step"))
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
    if(scene == nullptr) {
        scene = new QGraphicsScene();
        view_->setScene(scene);
        scene->installEventFilter(this);
    }

    view_->setFixedSize(QSize(state.width, state.height));
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
    rect_ = new QGraphicsRectItem(0, 0, 0, 0);
    rect_->setPen(QPen(Qt::red, 10.0 / view_->transform().m11()));
    rect_->setFlags(QGraphicsItem::ItemIsMovable);
    view_->scene()->addItem(pixmap_);
    view_->scene()->addItem(rect_);

    connect(this, SIGNAL(displayRequest(QSharedPointer<QImage>)), this, SLOT(display(QSharedPointer<QImage>)));
    connect(this, SIGNAL(submitRequest()), this, SLOT(submit()));
    connect(this, SIGNAL(dropRequest()), this, SLOT(drop()));

    DefaultNodeAdapter::setupUi(layout);
}

Memento::Ptr ImageRoiAdapter::getState() const
{
    return std::shared_ptr<State>(new State(state));
}

void ImageRoiAdapter::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<State> m = std::dynamic_pointer_cast<State> (memento);
    apex_assert_hard(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
    rect_->setPos(state.scene_pos);
    loaded_ = true;
}

void ImageRoiAdapter::display(QSharedPointer<QImage> img)
{
    QPixmap pixmap = QPixmap::fromImage(*img);
    state.roi_rect.setWidth(wrapped_->readParameter<int>("roi width"));
    state.roi_rect.setHeight(wrapped_->readParameter<int>("roi height"));

    QSize roi_size(state.roi_rect.width(), state.roi_rect.height());
    bool change = state.last_size != img->size() ||
                  state.last_roi_size != roi_size;

    if(change || loaded_) {
        view_->scene()->setSceneRect(img->rect());
        view_->fitInView(view_->sceneRect(), Qt::KeepAspectRatio);
        loaded_ = false;
    }

    if(pixmap_ != nullptr)
        pixmap_->setPixmap(pixmap);

    rect_->setRect(state.roi_rect);
    rect_->setZValue(pixmap_->zValue() + 0.1);

    if(change)
        submit();

    view_->scene()->update();

    state.last_size     = img->size();
    state.last_roi_size = roi_size;
    state.scene_pos     = rect_->scenePos();
}

void ImageRoiAdapter::fitInView()
{
    if(state.last_size.isNull()) {
        return;
    }
    state.width  = state.last_size.width();
    state.height = state.last_size.height();
    view_->setFixedSize(QSize(state.width, state.height));
    view_->fitInView(view_->sceneRect(), Qt::KeepAspectRatio);
}

void ImageRoiAdapter::submit()
{
    if(pixmap_ == nullptr)
        return;


    QPointF top_left     = rect_->sceneBoundingRect().topLeft();
    QPointF bottom_right = rect_->sceneBoundingRect().bottomRight();

    top_left.setX(std::min(pixmap_->pixmap().width(),
                           std::max(0, (int) top_left.x())));
    top_left.setY(std::min(pixmap_->pixmap().height(),
                           std::max(0, (int) top_left.y())));
    bottom_right.setX(std::min(pixmap_->pixmap().width(),
                               std::max(0, (int) bottom_right.x())));
    bottom_right.setY(std::min(pixmap_->pixmap().height(),
                               std::max(0, (int) bottom_right.y())));

    Roi roi(top_left.x(), top_left.y(),
            bottom_right.x() - top_left.x(),
            bottom_right.y() - top_left.y());

    if(roi.w() > 0 && roi.h() > 0) {
        connection_types::RoiMessage::Ptr result_msg(new connection_types::RoiMessage);
        result_msg->value = roi;
        wrapped_->setResult(result_msg);
    }
}

void ImageRoiAdapter::drop()
{
    connection_types::RoiMessage::Ptr result_msg(new connection_types::RoiMessage);
    wrapped_->setResult(result_msg);
}
/// MOC
#include "moc_image_roi_adapter.cpp"
