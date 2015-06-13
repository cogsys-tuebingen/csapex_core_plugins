/// HEADER
#include "filter_static_mask_adapter.h"

/// COMPONENT
#include "filter_static_mask_painter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_node_adapter.h>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(FilterStaticMaskAdapter, csapex::FilterStaticMask)


FilterStaticMaskAdapter::FilterStaticMaskAdapter(NodeWorkerWeakPtr worker, FilterStaticMask *node, WidgetController* widget_ctrl)
    : DefaultNodeAdapter(worker, widget_ctrl), wrapped_(node)
{
    // translate to UI thread via Qt signal
    wrapped_->show_painter.connect(std::bind(&FilterStaticMaskAdapter::displayRequest, this));
    wrapped_->input.connect(std::bind(&FilterStaticMaskAdapter::inputRequest, this, std::placeholders::_1));

    QObject::connect(this, SIGNAL(displayRequest()), this, SLOT(display()));
}

FilterStaticMaskAdapter::~FilterStaticMaskAdapter()
{
}

void FilterStaticMaskAdapter::setMask(cv::Mat mask)
{
    wrapped_->setMask(mask);
}

void FilterStaticMaskAdapter::display()
{
    StaticMaskPainter painter(wrapped_->getMask());

    QObject::connect(this, SIGNAL(inputRequest(cv::Mat)), &painter, SLOT(input(cv::Mat)));
    QObject::connect(&painter, SIGNAL(new_mask(cv::Mat)), this, SLOT(setMask(cv::Mat)));

    painter.run();
}
/// MOC
#include "moc_filter_static_mask_adapter.cpp"
