/// HEADER
#include "filter_static_mask_adapter.h"

/// COMPONENT
#include "filter_static_mask_painter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(FilterStaticMaskAdapter, csapex::FilterStaticMask)


FilterStaticMaskAdapter::FilterStaticMaskAdapter(NodeFacadeWeakPtr worker, NodeBox* parent, std::weak_ptr<FilterStaticMask> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node)
{
    qRegisterMetaType < cv::Mat > ("cv::Mat");

    auto n = wrapped_.lock();

    // translate to UI thread via Qt signal
    n->show_painter.connect(std::bind(&FilterStaticMaskAdapter::displayRequest, this));
    n->input.connect(std::bind(&FilterStaticMaskAdapter::inputRequest, this, std::placeholders::_1));

    QObject::connect(this, SIGNAL(displayRequest()), this, SLOT(display()));
}

FilterStaticMaskAdapter::~FilterStaticMaskAdapter()
{
}

void FilterStaticMaskAdapter::setMask(cv::Mat mask)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    node->setMask(mask);
}

void FilterStaticMaskAdapter::display()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    StaticMaskPainter painter(node->getMask());

    QObject::connect(this, SIGNAL(inputRequest(cv::Mat)), &painter, SLOT(input(cv::Mat)));
    QObject::connect(&painter, SIGNAL(new_mask(cv::Mat)), this, SLOT(setMask(cv::Mat)));

    painter.run();
}
/// MOC
#include "moc_filter_static_mask_adapter.cpp"
