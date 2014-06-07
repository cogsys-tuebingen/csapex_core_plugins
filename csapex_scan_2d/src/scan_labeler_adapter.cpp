/// HEADER
#include "scan_labeler_adapter.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/utility/register_node_adapter.h>
#include <utils_qt/QtCvImageConverter.h>
#include <csapex/utility/color.hpp>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(ScanLabelerAdapter, csapex::ScanLabeler)


ScanLabelerAdapter::ScanLabelerAdapter(ScanLabeler *node, WidgetController* widget_ctrl)
    : DefaultNodeAdapter(node, widget_ctrl), wrapped_(node), pixmap_(NULL), view_(new QGraphicsView), empty(32, 32, QImage::Format_RGB16), painter(&empty), down_(false)
{
    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));

    view_->setScene(new QGraphicsScene);

    // translate to UI thread via Qt signal
    node->display_request.connect(boost::bind(&ScanLabelerAdapter::displayRequest, this, _1));
    node->submit_request.connect(boost::bind(&ScanLabelerAdapter::submitRequest, this));
}


bool ScanLabelerAdapter::eventFilter(QObject *o, QEvent *e)
{
    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::GraphicsSceneMousePress:
        down_ = true;
        last_pos_ = me->screenPos();
        e->accept();
        return true;
    case QEvent::GraphicsSceneMouseRelease: {
        down_ = false;

        QGraphicsScene* scene = view_->scene();
        Q_FOREACH(QGraphicsItem* item, scene->selectedItems()) {
            QGraphicsEllipseItem* ellipse = dynamic_cast<QGraphicsEllipseItem*> (item);
            if(ellipse) {
                QBrush brush(color::fromCount(1), Qt::SolidPattern);
                ellipse->setBrush(brush);
            }
        }

        e->accept();
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

    default:
        break;
    }

    return false;
}

void ScanLabelerAdapter::setupUi(QBoxLayout* layout)
{
    view_->setFixedSize(QSize(state.width, state.height));
    view_->setMouseTracking(true);
    view_->setAcceptDrops(false);
    QGraphicsScene* scene = view_->scene();
    if(scene == NULL) {
        scene = new QGraphicsScene();
        view_->setScene(scene);
        scene->installEventFilter(this);
    }

    layout->addWidget(view_);

    connect(this, SIGNAL(displayRequest(lib_laser_processing::Scan* )), this, SLOT(display(lib_laser_processing::Scan* )));
    connect(this, SIGNAL(submitRequest()), this, SLOT(submit()));

    DefaultNodeAdapter::setupUi(layout);
}

Memento::Ptr ScanLabelerAdapter::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ScanLabelerAdapter::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

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

    QBrush brush(color::fromCount(0), Qt::SolidPattern);
    Q_FOREACH(const lib_laser_processing::LaserBeam& beam, scan->rays) {
        QGraphicsEllipseItem* item = scene->addEllipse(beam.pos(0), beam.pos(1), dim, dim, QPen(), brush);

        item->setFlag(QGraphicsItem::ItemIsSelectable);
    }


    scene->setSceneRect(rect);
    view_->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    scene->update();
}

void ScanLabelerAdapter::submit()
{
    std::cerr << "submit" << std::endl;
    wrapped_->setResult(result_);
}
