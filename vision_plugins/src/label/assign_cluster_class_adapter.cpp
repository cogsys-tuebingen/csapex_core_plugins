/// HEADER
#include "assign_cluster_class_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_node_adapter.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex/utility/color.hpp>
#include <csapex/utility/assert.h>
#include <utils_vision/utils/histogram.hpp>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>
#include <QKeyEvent>

using namespace csapex;
using namespace vision_plugins;

CSAPEX_REGISTER_NODE_ADAPTER_NS(vision_plugins, AssignClusterClassAdapter, vision_plugins::AssignClusterClass)

AssignClusterClassAdapter::AssignClusterClassAdapter(NodeWorkerWeakPtr worker, std::weak_ptr<AssignClusterClass> node, WidgetController* widget_ctrl)
    : DefaultNodeAdapter(worker, widget_ctrl),
      wrapped_(node),
      active_class_(0),
      pixmap_overlay_(nullptr),
      pixmap_(nullptr),
      view_(new QGraphicsView),
      empty(32, 32, QImage::Format_RGB16),
      painter(&empty),
      middle_button_down_(false),
      left_button_down_(false),
      loaded_(false)
{
    qRegisterMetaType < cv::Mat > ("cv::Mat");

    auto n = wrapped_.lock();

    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));

    // translate to UI thread via Qt signal
    trackConnection(n->display_request.connect(std::bind(&AssignClusterClassAdapter::displayRequest, this,
                                            std::placeholders::_1, std::placeholders::_2)));
    trackConnection(n->set_class.connect(std::bind(&AssignClusterClassAdapter::setClassRequest, this, std::placeholders::_1)));
    trackConnection(n->set_color.connect(std::bind(&AssignClusterClassAdapter::setColorRequest, this,
                                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)));
    trackConnection(n->submit_request.connect(std::bind(&AssignClusterClassAdapter::submitRequest, this)));
    trackConnection(n->drop_request.connect(std::bind(&AssignClusterClassAdapter::dropRequest, this)));
    trackConnection(n->clear_request.connect(std::bind(&AssignClusterClassAdapter::clearRequest, this)));
}

bool AssignClusterClassAdapter::eventFilter(QObject *o, QEvent *e)
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
            QPointF ppos = pixmap_->scenePos();
            QPointF mpos = me->scenePos();

            updateClusterClass(QPoint(mpos.x() - ppos.x(),
                                      mpos.y() - ppos.y()));

            left_button_down_ = false;
            e->accept();
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
            QPointF ppos = pixmap_->scenePos();
            QPointF mpos = me->scenePos();

            updateClusterClass(QPoint(mpos.x() - ppos.x(),
                                      mpos.y() - ppos.y()));

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

void AssignClusterClassAdapter::updateClusterClass(const QPoint &pos)
{
    if(pos.x() < 0 || pos.x() >= clusters_.cols)
        return;
    if(pos.y() < 0 || pos.y() >= clusters_.rows)
        return;

    int cluster = clusters_.at<int>(pos.y(),pos.x());
    classes_.at(cluster) = active_class_;

    if(overlay_.isNull()) {
        overlay_ = QImage(img_.size(), QImage::Format_ARGB32);
        overlay_.fill(QColor(0,0,0,0));
    }

    QColor &c = colors_[active_class_];
    for(int i = 0 ; i < clusters_.rows ; ++i) {
        for(int j = 0 ; j < clusters_.cols ; ++j) {
            if(clusters_.at<int>(i,j) == cluster) {
                overlay_.setPixel(j,i, c.rgba());
            }
        }
    }

    QPixmap pixmap = QPixmap::fromImage(overlay_);
    if(pixmap_overlay_ != nullptr)
        pixmap_overlay_->setPixmap(pixmap);

    view_->scene()->update();
}

void AssignClusterClassAdapter::setupUi(QBoxLayout* layout)
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

    pixmap_overlay_ = new QGraphicsPixmapItem;
    pixmap_ = new QGraphicsPixmapItem;
    view_->scene()->addItem(pixmap_);
    view_->scene()->addItem(pixmap_overlay_);

    connect(this, SIGNAL(displayRequest(QImage, const cv::Mat&)), this, SLOT(display(QImage, const cv::Mat&)));
    connect(this, SIGNAL(submitRequest()), this, SLOT(submit()));
    connect(this, SIGNAL(dropRequest()), this, SLOT(drop()));
    connect(this, SIGNAL(clearRequest()), this, SLOT(clear()));
    connect(this, SIGNAL(setColorRequest(int,int,int)), this, SLOT(setColor(int,int,int)));
    connect(this, SIGNAL(setClassRequest(int)), this, SLOT(setClass(int)));

    DefaultNodeAdapter::setupUi(layout);
}

Memento::Ptr AssignClusterClassAdapter::getState() const
{
    return std::shared_ptr<State>(new State(state));
}

void AssignClusterClassAdapter::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<State> m = std::dynamic_pointer_cast<State> (memento);
    apex_assert_hard(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
    loaded_ = true;
}

void AssignClusterClassAdapter::display(QImage img, const cv::Mat &clusters)
{
    /// PREPARE LABLES
    int num_clusters = utils_vision::histogram::numClusters(clusters);
    classes_.resize(num_clusters, -1);
    img_ = img;

    QPixmap pixmap = QPixmap::fromImage(img_);

    bool init_overlay = clusters_.rows != clusters.rows || clusters_.cols != clusters.cols;
    if(!clusters_.empty()) {
        cv::Mat diff;
        cv::subtract(clusters, clusters_, diff);
        init_overlay |= (cv::countNonZero(diff) > 0);
    }

    if(init_overlay) {
        overlay_ = QImage(img_.size(), QImage::Format_ARGB32);
        overlay_.fill(QColor(0,0,0,0));
        QPixmap pixmap = QPixmap::fromImage(overlay_);
        pixmap_overlay_->setPixmap(pixmap);
    }

    clusters_ = clusters;

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

void AssignClusterClassAdapter::fitInView()
{
    if(state.last_size.isNull()) {
        return;
    }
    state.width  = state.last_size.width();
    state.height = state.last_size.height();
    view_->setFixedSize(QSize(state.width, state.height));
    view_->fitInView(view_->sceneRect(), Qt::KeepAspectRatio);
}

void AssignClusterClassAdapter::submit()
{
    if(pixmap_ == nullptr)
        return;

    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    node->setResult(classes_);
}

void AssignClusterClassAdapter::drop()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    std::vector<int> empty;
    node->setResult(empty);
}

void AssignClusterClassAdapter::clear()
{
    if(!img_.isNull()) {
        classes_.resize(classes_.size(), -1);
        overlay_ = QImage(img_.size(), QImage::Format_ARGB32);
        overlay_.fill(QColor(0,0,0,0));
        QPixmap pixmap = QPixmap::fromImage(overlay_);
        pixmap_overlay_->setPixmap(pixmap);
    }
}

void AssignClusterClassAdapter::setColor(int r, int g, int b)
{
    QColor c(r,g,b,127);
    colors_[active_class_] = c;
}

void AssignClusterClassAdapter::setClass(int c)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    active_class_ = c;
    QColor &col = colors_[c];
    node->setActiveClassColor(col.red(),col.green(), col.blue());
}

/// MOC
#include "moc_assign_cluster_class_adapter.cpp"
