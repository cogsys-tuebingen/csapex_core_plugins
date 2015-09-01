/// HEADER
#include "point_count_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(PointCountAdapter, csapex::PointCount)

PointCountAdapter::PointCountAdapter(NodeWorkerWeakPtr worker, std::weak_ptr<PointCount> node, WidgetController* widget_ctrl)
    : NodeAdapter(worker, widget_ctrl), wrapped_(node)
{
    auto n = wrapped_.lock();

    trackConnection(n->display_request.connect(std::bind(&PointCountAdapter::display, this, std::placeholders::_1)));
}

void PointCountAdapter::setupUi(QBoxLayout* layout)
{
    number_ = new QLCDNumber;
    number_->setDigitCount(8);

    layout->addWidget(number_);

    QObject::connect(this, SIGNAL(displayRequest(int)), number_, SLOT(display(int)));
}

void PointCountAdapter::display(int img)
{
    Q_EMIT displayRequest(img);
}
/// MOC
#include "moc_point_count_adapter.cpp"
